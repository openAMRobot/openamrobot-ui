import { createSlice } from "@reduxjs/toolkit";

const initialState = {
  logs: [],
};

export const robotStore = createSlice({
  name: "robotStore",
  initialState,
  reducers: {
    setLogs: (state, action) => {
      console.log("Logs in store", state.logs, action.payload);
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
