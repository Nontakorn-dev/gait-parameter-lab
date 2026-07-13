import './gait-dashboard/styles/main.css';
import { createGaitLabDashboardApp } from './gait-dashboard/app.js';

const app = createGaitLabDashboardApp({
  teardownTransportOnDestroy: false,
});

app.init();

// Expose for manual debugging in the browser console (e.g. app.startBrowserBle()).
window.gaitLabApp = app;
