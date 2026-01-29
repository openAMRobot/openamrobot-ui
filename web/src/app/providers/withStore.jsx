import React from "react";
import { Provider } from "react-redux";
import { store } from "../store";

const withStore = (component) => () => (
  <Provider store={store}>{component()}</Provider>
);

export default withStore;
