import './debug.css';
import { DebugApp } from './debugApp.js';

const app = new DebugApp();
app.init();

window.addEventListener('beforeunload', () => app.destroy());
