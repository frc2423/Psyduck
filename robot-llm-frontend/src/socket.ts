import { useRobotStore } from './store'
import type { ClientEvent, ServerEvent } from './types'

const RECONNECT_MS = 1500

let ws: WebSocket | null = null

function url(): string {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws'
  return `${proto}://${location.host}/ws`
}

/** Open the bridge socket and keep it open; safe to call more than once. */
export function connect(): void {
  if (ws && ws.readyState <= WebSocket.OPEN) return
  useRobotStore.getState().setSocket('connecting')

  const socket = new WebSocket(url())
  ws = socket
  socket.onopen = () => useRobotStore.getState().setSocket('open')
  socket.onmessage = (ev) => {
    useRobotStore.getState().applyEvent(JSON.parse(ev.data) as ServerEvent)
  }
  socket.onclose = () => {
    useRobotStore.getState().setSocket('closed')
    // A dropped socket also means any in-flight turn and robot link are gone.
    useRobotStore.setState({ turnActive: false, robotConnected: false, agentReady: false })
    if (ws === socket) setTimeout(connect, RECONNECT_MS)
  }
  socket.onerror = () => socket.close()
}

export function send(event: ClientEvent): boolean {
  if (!ws || ws.readyState !== WebSocket.OPEN) return false
  ws.send(JSON.stringify(event))
  return true
}

export function sendChat(text: string): void {
  const store = useRobotStore.getState()
  if (send({ type: 'chat', text })) {
    store.addUserMessage(text)
  } else {
    store.applyEvent({ type: 'error', message: 'Not connected to the bridge server.' })
  }
}

export const cancelAll = () => send({ type: 'cancel_all' })
export const cancelCommand = (name: string) => send({ type: 'cancel', name })
