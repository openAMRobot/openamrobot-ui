import { combineReducers, configureStore } from "@reduxjs/toolkit";

import { robotReducer } from "../../stores";

const rootReducer = combineReducers({
  robotReducer,
});

export const store = configureStore({
  reducer: rootReducer,
});
