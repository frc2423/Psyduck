import Box from '@mui/material/Box'
import Chip from '@mui/material/Chip'
import Paper from '@mui/material/Paper'
import Stack from '@mui/material/Stack'
import Tooltip from '@mui/material/Tooltip'
import Typography from '@mui/material/Typography'
import { useRobotStore } from '../store'
import type { ParamSpec } from '../types'
import StatusChip from './StatusChip'

function paramLabel(p: ParamSpec): string {
  if (p.choices?.length) return `${p.name}: ${p.choices.join(' | ')}`
  const range = p.min !== null || p.max !== null ? ` [${p.min ?? '…'}..${p.max ?? '…'}]` : ''
  return `${p.name}: ${p.type}${range}`
}

export default function CommandsPanel() {
  const commands = useRobotStore((s) => s.commands)
  const commandStatus = useRobotStore((s) => s.commandStatus)

  return (
    <Paper variant="outlined" sx={{ p: 2 }}>
      <Typography variant="subtitle2" gutterBottom>
        Available commands {commands.length > 0 && `(${commands.length})`}
      </Typography>
      {commands.length === 0 ? (
        <Typography variant="body2" color="text.secondary">
          Waiting for the robot to publish its manifest.
        </Typography>
      ) : (
        <Stack spacing={1.5}>
          {commands.map((cmd) => {
            const st = commandStatus[cmd.name]
            return (
              <Box key={cmd.name}>
                <Stack direction="row" spacing={1} sx={{ alignItems: "center" }}>
                  <Typography sx={{ fontFamily: 'monospace', fontWeight: 600, flex: 1 }}>{cmd.name}</Typography>
                  {cmd.timeoutSeconds > 0 && (
                    <Typography variant="caption" color="text.secondary">
                      {cmd.timeoutSeconds.toFixed(0)}s
                    </Typography>
                  )}
                  <StatusChip status={st?.status ?? 'unknown'} />
                </Stack>
                <Typography variant="body2" color="text.secondary">
                  {cmd.description}
                </Typography>
                {cmd.parameters.length > 0 && (
                  <Stack direction="row" spacing={0.5} useFlexGap sx={{ flexWrap: 'wrap', mt: 0.5 }}>
                    {cmd.parameters.map((p) => (
                      <Tooltip key={p.name} title={p.description} arrow>
                        <Chip size="small" variant="outlined" label={paramLabel(p)} sx={{ fontFamily: 'monospace' }} />
                      </Tooltip>
                    ))}
                  </Stack>
                )}
                {st?.lastResult && st.status !== 'idle' && (
                  <Typography variant="caption" color="text.secondary" component="div" sx={{ mt: 0.5 }}>
                    last: {st.lastResult}
                  </Typography>
                )}
              </Box>
            )
          })}
        </Stack>
      )}
    </Paper>
  )
}
