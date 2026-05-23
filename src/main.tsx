import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import './index.css'
import App from './App.tsx'
import { installStateLatticePolicyWorker } from './planning/installStateLatticePolicyWorker'

const root = document.getElementById('root')

if (!root) {
  throw new Error('Root element #root was not found')
}

installStateLatticePolicyWorker()

createRoot(root).render(
  <StrictMode>
    <App />
  </StrictMode>,
)
