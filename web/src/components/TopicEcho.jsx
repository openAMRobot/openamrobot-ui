import React, { useEffect, useRef, useState } from "react";
import { useRos } from "../app/App";

const MAX_MESSAGES = 50;

/**
 * "Echo any topic by name" — the browser equivalent of `ros2 topic echo`.
 * The operator types a topic (and optionally its message type), and we open a
 * rosbridge subscription and stream the raw JSON messages. Message type is
 * optional: rosbridge can resolve it for an existing topic, but supplying it
 * avoids a lookup and works for topics that aren't advertised yet.
 */
const TopicEcho = () => {
  const ros = useRos();
  const [topicName, setTopicName] = useState("");
  const [msgType, setMsgType] = useState("");
  const [active, setActive] = useState(null); // { name, type }
  const [messages, setMessages] = useState([]);
  const [count, setCount] = useState(0);

  const topicRef = useRef(null);
  const seqRef = useRef(0);

  const stop = () => {
    if (topicRef.current) {
      try {
        topicRef.current.unsubscribe();
      } catch (e) {
        // ignore
      }
      topicRef.current = null;
    }
    setActive(null);
  };

  useEffect(() => stop, []); // unsubscribe on unmount

  const start = () => {
    const name = topicName.trim();
    if (!ros || !window.ROSLIB || !name) return;
    stop();
    setMessages([]);
    setCount(0);
    seqRef.current = 0;

    const type = msgType.trim();
    const topic = new window.ROSLIB.Topic({
      ros,
      name,
      // roslibjs tolerates an empty type on subscribe; rosbridge resolves it.
      messageType: type || undefined,
    });

    topic.subscribe((msg) => {
      setCount((c) => c + 1);
      setMessages((prev) => {
        const entry = {
          id: seqRef.current++,
          t: new Date().toLocaleTimeString(undefined, { hour12: false }),
          body: JSON.stringify(msg, null, 2),
        };
        const next = [entry, ...prev];
        return next.length > MAX_MESSAGES ? next.slice(0, MAX_MESSAGES) : next;
      });
    });

    topicRef.current = topic;
    setActive({ name, type });
  };

  return (
    <article className="dashboard-card flex h-full min-h-0 w-full flex-col overflow-hidden font-[RobotoMono]">
      <header className="flex flex-wrap items-center gap-2 border-b border-borderSubtle bg-bgSurface px-3 py-2">
        <h2 className="mr-1 text-sm font-semibold uppercase tracking-wider text-themeBlue">
          Topic Echo
        </h2>
        <p className="w-full text-[11px] text-themeTextGray">
          For advanced troubleshooting only — ask your integrator for the exact topic names to type here.
        </p>
        <input
          value={topicName}
          onChange={(e) => setTopicName(e.target.value)}
          onKeyDown={(e) => e.key === "Enter" && start()}
          placeholder="/topic_name"
          className="min-w-[140px] flex-1 rounded-lg border border-borderSubtle bg-bgCard px-2 py-1 text-xs text-textWhiteHover placeholder:text-themeTextGray"
        />
        <input
          value={msgType}
          onChange={(e) => setMsgType(e.target.value)}
          onKeyDown={(e) => e.key === "Enter" && start()}
          placeholder="msg type (optional)"
          className="w-40 rounded-lg border border-borderSubtle bg-bgCard px-2 py-1 text-xs text-textWhiteHover placeholder:text-themeTextGray"
        />
        {active ? (
          <button
            onClick={stop}
            className="rounded-lg border border-statusRed px-3 py-1 text-xs text-statusRed transition-colors hover:bg-statusRed hover:text-white"
          >
            Stop
          </button>
        ) : (
          <button
            onClick={start}
            className="rounded-lg border border-themeBlue px-3 py-1 text-xs text-themeBlue transition-colors hover:bg-themeBlue hover:text-white"
          >
            Echo
          </button>
        )}
      </header>

      <div className="flex-1 overflow-y-auto px-2 py-1 text-xs">
        {!active ? (
          <div className="flex h-full items-center justify-center px-4 text-center text-themeTextGray opacity-50">
            Enter a topic name and press Echo to stream its messages.
          </div>
        ) : messages.length === 0 ? (
          <div className="flex h-full items-center justify-center text-themeTextGray opacity-50">
            Subscribed to {active.name} — waiting for a message…
          </div>
        ) : (
          messages.map((m) => (
            <div key={m.id} className="border-b border-borderSubtle/20 py-1 last:border-0">
              <span className="text-[10px] uppercase text-themeTextGray opacity-70">
                {m.t}
              </span>
              <pre className="mt-0.5 overflow-x-auto whitespace-pre-wrap break-words text-[11px] text-textWhiteHover">
                {m.body}
              </pre>
            </div>
          ))
        )}
      </div>

      {active && (
        <footer className="border-t border-borderSubtle bg-bgSurface px-3 py-1 text-[10px] uppercase tracking-wider text-themeTextGray">
          {active.name} · {count} msgs
        </footer>
      )}
    </article>
  );
};

export default TopicEcho;
