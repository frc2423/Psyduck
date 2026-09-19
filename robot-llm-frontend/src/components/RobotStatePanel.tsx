import Box from '@mui/material/Box'
import Chip from '@mui/material/Chip'
import Paper from '@mui/material/Paper'
import Table from '@mui/material/Table'
import TableBody from '@mui/material/TableBody'
import TableCell from '@mui/material/TableCell'
import TableRow from '@mui/material/TableRow'
import Typography from '@mui/material/Typography'
import { useRobotStore } from '../store'
import type { StateValue } from '../types'

function Value({ v }: { v: StateValue }) {
  if (typeof v === 'boolean') {
    return <Chip size="small" label={v ? 'true' : 'false'} color={v ? 'success' : 'default'} variant="outlined" />
  }
  if (typeof v === 'number') {
    return <span>{Number.isInteger(v) ? v : v.toFixed(3)}</span>
  }
  return <span>{v}</span>
}

/** Group "drivetrain/x_meters" style keys by their first path segment. */
function groupByPrefix(state: Record<string, StateValue>): [string, [string, StateValue][]][] {
  const groups = new Map<string, [string, StateValue][]>()
  for (const [key, value] of Object.entries(state)) {
    const slash = key.indexOf('/')
    const group = slash === -1 ? '' : key.slice(0, slash)
    const leaf = slash === -1 ? key : key.slice(slash + 1)
    if (!groups.has(group)) groups.set(group, [])
    groups.get(group)!.push([leaf, value])
  }
  return [...groups.entries()].sort(([a], [b]) => a.localeCompare(b))
}

export default function RobotStatePanel() {
  const robotState = useRobotStore((s) => s.robotState)
  const robotConnected = useRobotStore((s) => s.robotConnected)
  const groups = groupByPrefix(robotState)

  return (
    <Paper variant="outlined" sx={{ p: 2 }}>
      <Typography variant="subtitle2" gutterBottom>
        Robot state
      </Typography>
      {groups.length === 0 ? (
        <Typography variant="body2" color="text.secondary">
          {robotConnected ? 'No telemetry published yet.' : 'Robot not connected.'}
        </Typography>
      ) : (
        groups.map(([group, entries]) => (
          <Box key={group} sx={{ mb: 1.5 }}>
            {group && (
              <Typography variant="overline" color="text.secondary" sx={{ lineHeight: 1.8 }}>
                {group}
              </Typography>
            )}
            <Table size="small" sx={{ '& td': { py: 0.25, border: 0 } }}>
              <TableBody>
                {entries.map(([leaf, value]) => (
                  <TableRow key={leaf}>
                    <TableCell sx={{ fontFamily: 'monospace', color: 'text.secondary', pl: 0, width: '55%' }}>
                      {leaf}
                    </TableCell>
                    <TableCell align="right" sx={{ fontFamily: 'monospace', pr: 0 }}>
                      <Value v={value} />
                    </TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          </Box>
        ))
      )}
    </Paper>
  )
}
