import "@testing-library/jest-dom";
import React from "react";
import { act, render, screen, within } from "@testing-library/react";

import CurrentRunFixtureView from "./CurrentRunFixtureView";
import * as missionRunner from "../shared/missions/missionRunner";
import { installDemoTransport, startDemoTicking, uninstallDemoTransport } from "../shared/demo/demoData";
import { createRunStateAdapter } from "../shared/runState/runStateAdapter";
import { FIXTURE_PREFIX, createRunStateFixtureTransport } from "../shared/runState/runStateFixtureTransport";
import {
  STEP_FAILED,
  createFakeProducer,
  createFakeTimers,
  created,
  ended,
  flush,
  started,
} from "../shared/runState/testFixtures";

const makeAdapter = (transport) => {
  const timers = createFakeTimers();
  let now = 0;
  const adapter = createRunStateAdapter({
    transport,
    now: () => now,
    timers,
    pollMs: 1000,
    staleAfterMs: 3000,
  });
  return {
    adapter,
    timers,
    advance: (ms) => {
      now += ms;
    },
    poll: async () => {
      await act(async () => {
        timers.runAll();
        await flush();
      });
    },
  };
};

const renderView = async (adapter) => {
  let utils;
  await act(async () => {
    utils = render(<CurrentRunFixtureView adapter={adapter} />);
    await flush();
  });
  return utils;
};

// Any real-command attempt fails the test: ROSLIB constructors and the
// mission runner command channel are replaced by throwing spies.
let commandSpies;
beforeEach(() => {
  const forbidden = (name) =>
    jest.fn(() => {
      throw new Error(`forbidden command path: ${name}`);
    });
  window.ROSLIB = {
    Ros: forbidden("ROSLIB.Ros"),
    Topic: forbidden("ROSLIB.Topic"),
    Service: forbidden("ROSLIB.Service"),
    ActionClient: forbidden("ROSLIB.ActionClient"),
    Message: forbidden("ROSLIB.Message"),
  };
  commandSpies = [
    jest.spyOn(missionRunner, "requestStart").mockImplementation(() => {
      throw new Error("forbidden: requestStart");
    }),
    jest.spyOn(missionRunner, "requestStop").mockImplementation(() => {
      throw new Error("forbidden: requestStop");
    }),
    jest.spyOn(missionRunner, "setRun").mockImplementation(() => {
      throw new Error("forbidden: setRun");
    }),
  ];
});

afterEach(() => {
  jest.restoreAllMocks();
  delete window.ROSLIB;
  delete global.fetch;
});

const expectNoCommands = () => {
  commandSpies.forEach((spy) => expect(spy).not.toHaveBeenCalled());
  Object.values(window.ROSLIB).forEach((ctor) => expect(ctor).not.toHaveBeenCalled());
};

test("renders identity, step, state, separate outcome/reason and the local test-data label", async () => {
  const producer = createFakeProducer();
  producer.push(created(1, "fixture-run-0002"));
  const { adapter, poll } = makeAdapter(producer.transport);
  const { unmount } = await renderView(adapter);

  expect(screen.getByTestId("dev-test-data-label")).toHaveTextContent("Development test data");
  expect(screen.queryByText(/demo mode/i)).not.toBeInTheDocument();
  expect(screen.getByTestId("operational-state")).toHaveTextContent(/^READY$/);
  expect(screen.getByTestId("outcome")).toHaveTextContent("no terminal outcome");

  producer.push(started(2, "fixture-run-0002", 1000));
  await poll();
  expect(screen.getByTestId("operational-state")).toHaveTextContent(/^EXECUTING$/);
  expect(screen.getByTestId("step")).toHaveTextContent("step-1 (1 of 3)");

  producer.t = 6000;
  producer.push(ended(3, "fixture-run-0002", 6000, "FAILED", "NEEDS_ASSISTANCE", STEP_FAILED));
  await poll();
  expect(screen.getByTestId("run-id")).toHaveTextContent("fixture-run-0002");
  expect(screen.getByTestId("operational-state")).toHaveTextContent(/^NEEDS_ASSISTANCE$/);
  expect(screen.getByTestId("outcome")).toHaveTextContent(/^FAILED$/);
  expect(screen.getByTestId("outcome-reason")).toHaveTextContent("FIXTURE_PROVISIONAL.STEP_FAILED");
  expect(screen.getByTestId("elapsed")).toHaveTextContent("5 s (producer clock, final)");
  expect(screen.queryAllByRole("button")).toHaveLength(0);

  unmount();
  expect(adapter.listenerCount()).toBe(0);
  expectNoCommands();
});

test("404 shows 'Fixture backend disabled', distinct from disconnected, with no buttons", async () => {
  global.fetch = jest.fn(async () => ({ status: 404, json: async () => ({ code: 404 }) }));
  const { adapter } = makeAdapter(createRunStateFixtureTransport());
  const { unmount } = await renderView(adapter);
  expect(screen.getByTestId("fixture-disabled")).toHaveTextContent("Fixture backend disabled");
  expect(screen.queryByText(/disconnected/i)).not.toBeInTheDocument();
  expect(screen.queryByTestId("operational-state")).not.toBeInTheDocument();
  expect(screen.queryAllByRole("button")).toHaveLength(0);
  unmount();
  expectNoCommands();
});

test("Demo Mode on + fixture on + Flask disconnect: the view reports the disconnect", async () => {
  // Demo Mode's simulated rosbridge: healthy and ticking synthetic telemetry.
  const demoRos = { isConnected: true, callOnConnection: jest.fn(), emit: jest.fn() };
  installDemoTransport(demoRos);
  const stopTicking = startDemoTicking(demoRos);
  expect(demoRos.emit).toHaveBeenCalled();

  // Fixture served by Flask through the real transport; first up, then down.
  const producer = createFakeProducer();
  producer.push(created(1, "fixture-run-0001"));
  global.fetch = jest.fn(async (url, init) => {
    const u = new URL(url, "http://localhost");
    const res = u.pathname.endsWith("/snapshot")
      ? await producer.transport.fetchSnapshot()
      : await producer.transport.fetchEvents({
          epoch: u.searchParams.get("epoch"),
          afterSeq: Number(u.searchParams.get("after_seq")),
        });
    return { status: res.status, json: async () => res.body };
  });
  const { adapter, poll, advance } = makeAdapter(createRunStateFixtureTransport());
  const { unmount } = await renderView(adapter);
  expect(screen.getByTestId("connection-status")).toHaveTextContent("Connected to fixture backend");

  producer.mode = "down"; // Flask connection lost
  advance(4000);
  await poll();

  expect(screen.getByTestId("connection-status")).toHaveTextContent("Disconnected from fixture backend");
  expect(screen.getByTestId("freshness-status")).toHaveTextContent("Freshness: STALE");
  expect(screen.getByTestId("operational-state")).toHaveTextContent("last reported — not current");
  expect(screen.getByTestId("allowed-operations")).toHaveTextContent("unknown");
  expect(demoRos.isConnected).toBe(true); // demo still "green" — and did not mask it

  // Only GETs to the fixture prefix; nothing else went over fetch.
  global.fetch.mock.calls.forEach(([url, init]) => {
    expect(url).toContain(FIXTURE_PREFIX);
    expect(init.method).toBe("GET");
  });

  unmount();
  stopTicking();
  uninstallDemoTransport(demoRos);
  expectNoCommands();
});

test("two views on one adapter agree, share one poll loop, and leave no listeners or timers", async () => {
  const producer = createFakeProducer();
  producer.push(created(1, "fixture-run-0001"));
  const { adapter, poll, timers } = makeAdapter(producer.transport);
  let utils;
  await act(async () => {
    utils = render(
      <>
        <div data-testid="a"><CurrentRunFixtureView adapter={adapter} /></div>
        <div data-testid="b"><CurrentRunFixtureView adapter={adapter} /></div>
      </>,
    );
    await flush();
  });
  expect(adapter.listenerCount()).toBe(2);
  expect(producer.calls.snapshot).toBe(1);

  producer.push(started(2, "fixture-run-0001", 10));
  await poll();
  const a = within(screen.getByTestId("a"));
  const b = within(screen.getByTestId("b"));
  expect(a.getByTestId("current-run-fixture-view").textContent).toBe(
    b.getByTestId("current-run-fixture-view").textContent,
  );
  expect(a.getByTestId("operational-state")).toHaveTextContent(/^EXECUTING$/);

  // Reconnect (refresh) mid-run: same run, snapshot + newer events, no commands.
  await act(async () => {
    adapter.reconnect();
    await flush();
  });
  expect(a.getByTestId("run-id")).toHaveTextContent("fixture-run-0001");

  utils.unmount();
  expect(adapter.listenerCount()).toBe(0);
  expect(timers.pendingCount()).toBe(0);
  expectNoCommands();
});
