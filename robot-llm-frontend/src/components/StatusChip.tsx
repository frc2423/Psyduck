import Chip, { type ChipProps } from '@mui/material/Chip'
import type { CommandState } from '../types'

const COLORS: Record<CommandState, ChipProps['color']> = {
  idle: 'default',
  running: 'info',
  finished: 'success',
  interrupted: 'warning',
  rejected: 'error',
  unknown: 'default',
}

export default function StatusChip({ status, size = 'small' }: { status: CommandState; size?: ChipProps['size'] }) {
  return (
    <Chip
      label={status}
      size={size}
      color={COLORS[status] ?? 'default'}
      variant={status === 'idle' || status === 'unknown' ? 'outlined' : 'filled'}
      sx={{ fontFamily: 'monospace', textTransform: 'uppercase', fontSize: 11 }}
    />
  )
}
