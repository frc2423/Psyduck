import { create } from 'zustand'
import type {
  ChatMessage,
  CommandSpec,
  CommandStatus,
  FinishedCommand,
  ServerEvent,
  StateValue,
} from './types'

export type SocketState = 'connecting' | 'open' | 'closed'

interface RobotStore {
  socket: SocketState
  robotConnected: boolean
  agentReady: boolean
  model: string
  commands: CommandSpec[]
  commandStatus: Record<string, CommandStatus>
  robotState: Record<string, StateValue>
  lastFinished: FinishedCommand | null
  messages: ChatMessage[]
  turnActive: boolean

  setSocket: (socket: SocketState) => void
  addUserMessage: (text: string) => void
  applyEvent: (event: ServerEvent) => void
}

let nextId = 1
function stamp<T extends object>(m: T): T & { id: number; at: number } {
  return { ...m, id: nextId++, at: Date.now() }
}

/** Tool results start with the outcome word, e.g. "OK: ..." or "REJECTED: ...". */
const FAILURE_PREFIX = /^(REJECTED|INTERRUPTED|TIMEOUT|DISCONNECTED|UNKNOWN)\b/

export const useRobotStore = create<RobotStore>((set, get) => ({
  socket: 'connecting',
  robotConnected: false,
  agentReady: false,
  model: '',
  commands: [],
  commandStatus: {},
  robotState: {},
  lastFinished: null,
  messages: [],
  turnActive: false,

  setSocket: (socket) => set({ socket }),

  addUserMessage: (text) =>
    set((s) => ({ messages: [...s.messages, stamp({ role: 'user' as const, text })] })),

  applyEvent: (event) => {
    switch (event.type) {
      case 'hello':
        set({ model: event.model, commands: event.commands })
        break
      case 'manifest':
        set({ commands: event.commands })
        break
      case 'snapshot': {
        // A completion shows up as completedCount ticking over.
        const prev = get().commandStatus
        let lastFinished = get().lastFinished
        for (const [name, status] of Object.entries(event.commands)) {
          const before = prev[name]?.completedCount
          if (before !== undefined && status.completedCount > before) {
            lastFinished = { name, status: status.status, lastResult: status.lastResult, at: Date.now() }
          }
        }
        set({
          robotConnected: event.robotConnected,
          agentReady: event.agentReady,
          robotState: event.state,
          commandStatus: event.commands,
          lastFinished,
        })
        break
      }
      case 'turn':
        set({ turnActive: event.phase === 'start' })
        break
      case 'message': {
        let m: ChatMessage
        if (event.role === 'assistant') {
          m = stamp({ role: 'assistant' as const, text: event.text })
        } else if (event.role === 'tool_call') {
          m = stamp({ role: 'tool_call' as const, name: event.name, args: event.args })
        } else {
          m = stamp({
            role: 'tool_result' as const,
            name: event.name,
            content: event.content,
            ok: event.status === 'success' && !FAILURE_PREFIX.test(event.content),
          })
        }
        set((s) => ({ messages: [...s.messages, m] }))
        break
      }
      case 'error':
        set((s) => ({ messages: [...s.messages, stamp({ role: 'error' as const, text: event.message })] }))
        break
    }
  },
}))
