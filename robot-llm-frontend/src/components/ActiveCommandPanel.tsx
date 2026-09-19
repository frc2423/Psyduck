import { useEffect, useState } from 'react'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import LinearProgress from '@mui/material/LinearProgress'
import Paper from '@mui/material/Paper'
import Stack from '@mui/material/Stack'
import Typography from '@mui/material/Typography'
import StopIcon from '@mui/icons-material/Stop'
import { useRobotStore } from '../store'
import { cancelCommand } from '../socket'
import StatusChip from './StatusChip'

/** Re-render on a timer so elapsed-time readouts tick. */
function useClock(intervalMs: number, enabled: boolean): number {
  const [now, setNow] = useState(() => Date.now())
  useEffect(() => {
    if (!enabled) return
    const id = setInterval(() => setNow(Date.now()), intervalMs)
    return () => clearInterval(id)
  }, [intervalMs, enabled])
  return now
}

export default function ActiveCommandPanel() {
  const commandStatus = useRobotStore((s) => s.commandStatus)
  const commands = useRobotStore((s) => s.commands)
  const lastFinished = useRobotStore((s) => s.lastFinished)

  const running = Object.entries(commandStatus).filter(([, st]) => st.status === 'running')
  const now = useClock(200, running.length > 0)

  return (
    <Paper variant="outlined" sx={{ p: 2 }}>
      <Typography variant="subtitle2" gutterBottom>
        Running command
      </Typography>

      {running.length === 0 ? (
        <Typography variant="body2" color="text.secondary">
          Nothing running.
        </Typography>
      ) : (
        <Stack spacing={1.5}>
          {running.map(([name, st]) => {
            const spec = commands.find((c) => c.name === name)
            const elapsed = st.runningSince ? Math.max(0, now / 1000 - st.runningSince) : 0
            const timeout = spec?.timeoutSeconds ?? 0
            const progress = timeout > 0 ? Math.min(100, (elapsed / timeout) * 100) : null
            return (
              <Box key={name}>
                <Stack direction="row" spacing={1} sx={{ alignItems: "center" }}>
                  <Typography sx={{ fontFamily: 'monospace', fontWeight: 600, flex: 1 }}>{name}</Typography>
                  <StatusChip status="running" />
                  <Button
                    size="small"
                    color="warning"
                    variant="outlined"
                    startIcon={<StopIcon />}
                    onClick={() => cancelCommand(name)}
                  >
                    Cancel
                  </Button>
                </Stack>
                <Typography variant="caption" color="text.secondary">
                  {elapsed.toFixed(1)}s elapsed{timeout > 0 ? ` · ${timeout.toFixed(0)}s timeout` : ''}
                </Typography>
                {progress === null ? (
                  <LinearProgress sx={{ mt: 0.5 }} />
                ) : (
                  <LinearProgress variant="determinate" value={progress} sx={{ mt: 0.5 }} />
                )}
              </Box>
            )
          })}
        </Stack>
      )}

      {lastFinished && (
        <Box sx={{ mt: 2, pt: 1.5, borderTop: 1, borderColor: 'divider' }}>
          <Typography variant="caption" color="text.secondary" component="div">
            Last completed
          </Typography>
          <Stack direction="row" spacing={1} sx={{ alignItems: "center", mt: 0.5 }}>
            <Typography sx={{ fontFamily: 'monospace', flex: 1 }}>{lastFinished.name}</Typography>
            <StatusChip status={lastFinished.status} />
          </Stack>
          {lastFinished.lastResult && (
            <Typography variant="body2" color="text.secondary" sx={{ mt: 0.5 }}>
              {lastFinished.lastResult}
            </Typography>
          )}
        </Box>
      )}
    </Paper>
  )
}
