import { useEffect } from 'react'
import AppBar from '@mui/material/AppBar'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Chip from '@mui/material/Chip'
import Stack from '@mui/material/Stack'
import Toolbar from '@mui/material/Toolbar'
import Typography from '@mui/material/Typography'
import StopCircleIcon from '@mui/icons-material/StopCircle'
import { useRobotStore } from './store'
import { cancelAll, connect } from './socket'
import ChatPanel from './components/ChatPanel'
import ActiveCommandPanel from './components/ActiveCommandPanel'
import RobotStatePanel from './components/RobotStatePanel'
import CommandsPanel from './components/CommandsPanel'

function Indicator({ label, ok, pending }: { label: string; ok: boolean; pending?: boolean }) {
  return (
    <Chip
      size="small"
      label={label}
      color={ok ? 'success' : pending ? 'warning' : 'error'}
      variant={ok ? 'filled' : 'outlined'}
    />
  )
}

export default function App() {
  const socket = useRobotStore((s) => s.socket)
  const robotConnected = useRobotStore((s) => s.robotConnected)
  const agentReady = useRobotStore((s) => s.agentReady)
  const model = useRobotStore((s) => s.model)

  useEffect(() => {
    connect()
  }, [])

  return (
    <Box sx={{ display: 'flex', flexDirection: 'column', height: '100%', bgcolor: 'background.default' }}>
      <AppBar position="static" color="default" elevation={0} sx={{ borderBottom: 1, borderColor: 'divider' }}>
        <Toolbar variant="dense" sx={{ gap: 2 }}>
          <Typography variant="h6" sx={{ flexShrink: 0 }}>
            Robot LLM
          </Typography>
          <Stack direction="row" spacing={1} sx={{ flex: 1 }}>
            <Indicator label="bridge" ok={socket === 'open'} pending={socket === 'connecting'} />
            <Indicator label="robot" ok={robotConnected} />
            <Indicator label={model ? `agent · ${model}` : 'agent'} ok={agentReady} />
          </Stack>
          <Button
            color="error"
            variant="contained"
            startIcon={<StopCircleIcon />}
            disabled={!robotConnected}
            onClick={() => cancelAll()}
          >
            Stop all
          </Button>
        </Toolbar>
      </AppBar>

      <Box
        sx={{
          flex: 1,
          minHeight: 0,
          p: 2,
          display: 'grid',
          gap: 2,
          gridTemplateColumns: { xs: '1fr', md: 'minmax(0, 3fr) minmax(320px, 2fr)' },
          gridTemplateRows: { xs: 'minmax(50vh, 1fr) auto', md: 'minmax(0, 1fr)' },
        }}
      >
        <ChatPanel />
        <Stack spacing={2} sx={{ overflowY: 'auto', minHeight: 0 }}>
          <ActiveCommandPanel />
          <RobotStatePanel />
          <CommandsPanel />
        </Stack>
      </Box>
    </Box>
  )
}
