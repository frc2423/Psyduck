/** Wire types shared with `src/robot_llm/server.py`. */

export type ParamType = 'double' | 'integer' | 'boolean' | 'string'

export interface ParamSpec {
  name: string
  type: ParamType
  description: string
  min: number | null
  max: number | null
  choices: string[] | null
}

export interface CommandSpec {
  name: string
  description: string
  timeoutSeconds: number
  parameters: ParamSpec[]
}

export type CommandState = 'idle' | 'running' | 'finished' | 'interrupted' | 'rejected' | 'unknown'

export interface CommandStatus {
  status: CommandState
  lastResult: string
  completedCount: number
  /** Unix seconds when the server first saw the command running, or null. */
  runningSince: number | null
}

export type StateValue = number | boolean | string

export type ServerEvent =
  | { type: 'hello'; model: string; commands: CommandSpec[] }
  | { type: 'manifest'; commands: CommandSpec[] }
  | {
      type: 'snapshot'
      robotConnected: boolean
      agentReady: boolean
      state: Record<string, StateValue>
      commands: Record<string, CommandStatus>
    }
  | { type: 'turn'; phase: 'start' | 'end' }
  | { type: 'message'; role: 'assistant'; text: string }
  | { type: 'message'; role: 'tool_call'; id: string; name: string; args: Record<string, unknown> }
  | {
      type: 'message'
      role: 'tool_result'
      toolCallId: string
      name: string | null
      content: string
      status: 'success' | 'error'
    }
  | { type: 'error'; message: string }

export type ClientEvent =
  | { type: 'chat'; text: string }
  | { type: 'cancel_all' }
  | { type: 'cancel'; name: string }

export type ChatMessage = { id: number; at: number } & (
  | { role: 'user'; text: string }
  | { role: 'assistant'; text: string }
  | { role: 'tool_call'; name: string; args: Record<string, unknown> }
  | { role: 'tool_result'; name: string | null; content: string; ok: boolean }
  | { role: 'error'; text: string }
)

export interface FinishedCommand {
  name: string
  status: CommandState
  lastResult: string
  at: number
}
