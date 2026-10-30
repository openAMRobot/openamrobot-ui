import React from "react";
import { render, screen } from "@testing-library/react";
import "@testing-library/jest-dom";

import Camera from "./Camera";

jest.mock("../app/App", () => ({
  useRuntimeConfig: () => ({ config: { cameraPort: "8080" } }),
}));

jest.mock("../shared/constants/runtimeConfig", () => ({
  resolveRosbridgeHost: () => "localhost",
}));

it("lists the head camera as not reported while no topic exists", () => {
  render(<Camera />);
  const option = screen.getByRole("option", { name: "Head camera: not reported" });
  expect(option).toBeDisabled();
});
