import { createSlice } from "@reduxjs/toolkit";

const initialState = {
  logs: [],
};

export const robotStore = createSlice({
  name: "robotStore",
  initialState,
  reducers: {
    setLogs: (state, action) => {
      state.logs = action.payload;
    },
    addSingleLog: (state, action) => {
      state.logs.unshift(action.payload);
    },
  },
});

export const { setLogs, addSingleLog } = robotStore.actions;

export const logsSelector = (state) => state.robotReducer;

export const robotReducer = robotStore.reducer;
