import { useEffect, useRef, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import IconButton from '@mui/material/IconButton'
import LinearProgress from '@mui/material/LinearProgress'
import Paper from '@mui/material/Paper'
import Stack from '@mui/material/Stack'
import TextField from '@mui/material/TextField'
import Typography from '@mui/material/Typography'
import SendIcon from '@mui/icons-material/Send'
import { useRobotStore } from '../store'
import { sendChat } from '../socket'
import type { ChatMessage } from '../types'

function formatArgs(args: Record<string, unknown>): string {
  return Object.entries(args)
    .map(([k, v]) => `${k}=${JSON.stringify(v)}`)
    .join(', ')
}

function Message({ m }: { m: ChatMessage }) {
  switch (m.role) {
    case 'user':
      return (
        <Paper
          elevation={0}
          sx={{ alignSelf: 'flex-end', maxWidth: '80%', px: 1.5, py: 1, bgcolor: 'primary.main', color: 'primary.contrastText' }}
        >
          <Typography variant="body2" sx={{ whiteSpace: 'pre-wrap' }}>{m.text}</Typography>
        </Paper>
      )
    case 'assistant':
      return (
        <Paper variant="outlined" sx={{ alignSelf: 'flex-start', maxWidth: '80%', px: 1.5, py: 1 }}>
          <Typography variant="body2" sx={{ whiteSpace: 'pre-wrap' }}>{m.text}</Typography>
        </Paper>
      )
    case 'tool_call':
      return (
        <Typography variant="caption" component="div" sx={{ fontFamily: 'monospace', color: 'text.secondary', pl: 1 }}>
          → {m.name}({formatArgs(m.args)})
        </Typography>
      )
    case 'tool_result':
      return (
        <Typography
          variant="caption"
          component="div"
          sx={{ fontFamily: 'monospace', color: m.ok ? 'success.main' : 'warning.main', pl: 1, whiteSpace: 'pre-wrap' }}
        >
          ← {m.content}
        </Typography>
      )
    case 'error':
      return <Alert severity="error" variant="outlined" sx={{ py: 0 }}>{m.text}</Alert>
  }
}

export default function ChatPanel() {
  const messages = useRobotStore((s) => s.messages)
  const turnActive = useRobotStore((s) => s.turnActive)
  const agentReady = useRobotStore((s) => s.agentReady)
  const socketOpen = useRobotStore((s) => s.socket === 'open')
  const [draft, setDraft] = useState('')
  const bottomRef = useRef<HTMLDivElement>(null)

  useEffect(() => {
    bottomRef.current?.scrollIntoView({ behavior: 'smooth' })
  }, [messages.length, turnActive])

  const canSend = socketOpen && agentReady && !turnActive && draft.trim().length > 0

  const submit = () => {
    if (!canSend) return
    sendChat(draft.trim())
    setDraft('')
  }

  const placeholder = !socketOpen
    ? 'Connecting to bridge server…'
    : !agentReady
      ? 'Waiting for the robot to publish its commands…'
      : turnActive
        ? 'Agent is working…'
        : 'Tell the robot what to do (e.g. "drive 2 meters forward then score high")'

  return (
    <Paper variant="outlined" sx={{ display: 'flex', flexDirection: 'column', height: '100%', minHeight: 0 }}>
      <Box sx={{ px: 2, pt: 1.5, pb: 1, borderBottom: 1, borderColor: 'divider' }}>
        <Typography variant="subtitle2">Chat</Typography>
      </Box>

      <Stack spacing={1} sx={{ flex: 1, overflowY: 'auto', p: 2, minHeight: 0 }}>
        {messages.length === 0 && (
          <Typography variant="body2" color="text.secondary">
            Instructions you type here are sent to the LLM agent, which drives the robot by calling its
            commands. Tool calls and results are shown inline.
          </Typography>
        )}
        {messages.map((m) => (
          <Message key={m.id} m={m} />
        ))}
        <div ref={bottomRef} />
      </Stack>

      {turnActive && <LinearProgress />}

      <Box
        component="form"
        onSubmit={(e) => {
          e.preventDefault()
          submit()
        }}
        sx={{ display: 'flex', gap: 1, p: 1.5, borderTop: 1, borderColor: 'divider' }}
      >
        <TextField
          fullWidth
          size="small"
          multiline
          maxRows={4}
          placeholder={placeholder}
          value={draft}
          disabled={!socketOpen}
          onChange={(e) => setDraft(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === 'Enter' && !e.shiftKey) {
              e.preventDefault()
              submit()
            }
          }}
        />
        <IconButton type="submit" color="primary" disabled={!canSend} aria-label="send">
          <SendIcon />
        </IconButton>
      </Box>
    </Paper>
  )
}
