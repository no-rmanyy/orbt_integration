#!/usr/bin/env python3
import argparse
import socket
import sys
from typing import Dict, Optional, Tuple, Any, List

try:
	import cbor2
except Exception as e:
	print("This tool requires the 'cbor2' package. Install with: pip install cbor2", file=sys.stderr)
	raise


class TelemetryCborReceiver:
	def __init__(self) -> None:
		# id -> (key, is_float, unit)
		self.id_map: Dict[int, Tuple[str, bool, Optional[str]]] = {}

	def handle_packet(self, data: bytes, addr: Tuple[str, int], verbose: bool = False) -> None:
		try:
			msg = cbor2.loads(data)
		except Exception as e:
			print(f"[{addr[0]}] Failed to decode CBOR: {e}")
			return

		# Expect one of:
		# ["K", [ [key, id, isFloat(0/1), unit|null], ... ]]
		# ["V", timestamp_ms, [ [id, value], ... ]]
		# ["L", message]
		if not isinstance(msg, list) or not msg:
			print(f"[{addr[0]}] Unexpected message format: {type(msg)} {msg!r}")
			return

		tag = msg[0]
		if tag == "K":
			self._handle_keys(msg, verbose)
		elif tag == "V":
			self._handle_values(msg)
		elif tag == "L":
			self._handle_log(msg)
		else:
			print(f"[{addr[0]}] Unknown tag {tag!r}: {msg!r}")

	def _handle_keys(self, msg: List[Any], verbose: bool) -> None:
		if len(msg) != 2 or not isinstance(msg[1], list):
			print(f"Invalid K message: {msg!r}")
			return
		entries = msg[1]
		updated = 0
		for entry in entries:
			if not (isinstance(entry, list) and (3 <= len(entry) <= 4)):
				continue
			key = entry[0]
			idv = entry[1]
			is_float_flag = entry[2]
			unit = entry[3] if len(entry) >= 4 else None
			if not isinstance(key, str) or not isinstance(idv, int):
				continue
			is_float = bool(is_float_flag)
			if isinstance(unit, str):
				unit_str: Optional[str] = unit
			else:
				unit_str = None
			self.id_map[idv] = (key, is_float, unit_str)
			updated += 1
		if verbose:
			print(f"[K] Updated {updated} mapping(s). Current map:")
			for idv in sorted(self.id_map.keys()):
				key, is_float, unit = self.id_map[idv]
				unit_disp = unit if unit else ""
				print(f"  id={idv:3d}  key={key!r}  type={'float' if is_float else 'int'}  unit={unit_disp!r}")

	def _handle_values(self, msg: List[Any]) -> None:
		if len(msg) != 3 or not isinstance(msg[2], list):
			print(f"Invalid V message: {msg!r}")
			return
		ts_ms = msg[1]
		entries = msg[2]
		if not isinstance(ts_ms, int):
			print(f"Invalid timestamp in V message: {msg!r}")
			return

		out: List[str] = []
		for e in entries:
			if not (isinstance(e, list) and len(e) == 2):
				continue
			idv, val = e
			if not isinstance(idv, int):
				continue
			key, unit = None, None
			if idv in self.id_map:
				k, _is_float, u = self.id_map[idv]
				key = k
				unit = u
			name = key if key is not None else f"id{ idv }"
			if isinstance(val, float):
				val_str = f"{val:.6f}"
			else:
				val_str = f"{val}"
			if unit:
				out.append(f"{name}={val_str} {unit}")
			else:
				out.append(f"{name}={val_str}")
		out_str = ", ".join(out)
		print(f"[V t={ts_ms}] {out_str}")

	def _handle_log(self, msg: List[Any]) -> None:
		if len(msg) != 2:
			print(f"Invalid L message: {msg!r}")
			return
		text = msg[1]
		if not isinstance(text, str):
			text = repr(text)
		print(f"[L] {text}")


def main() -> None:
	parser = argparse.ArgumentParser(description="Receive CBOR telemetry over UDP and print it.")
	parser.add_argument("--port", type=int, default=1202, help="UDP port to listen on (default: 1202)")
	parser.add_argument("--bind", type=str, default="0.0.0.0", help="Bind address (default: 0.0.0.0)")
	parser.add_argument("-v", "--verbose", action="store_true", help="Verbose mapping output on K messages")
	args = parser.parse_args()

	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	try:
		sock.bind((args.bind, args.port))
	except OSError as e:
		print(f"Failed to bind {args.bind}:{args.port} - {e}", file=sys.stderr)
		sys.exit(1)

	print(f"Listening for CBOR telemetry on {args.bind}:{args.port} (Ctrl-C to quit)")
	receiver = TelemetryCborReceiver()
	try:
		while True:
			data, addr = sock.recvfrom(8192)
			receiver.handle_packet(data, addr, verbose=args.verbose)
	except KeyboardInterrupt:
		print("\nExiting.")


if __name__ == "__main__":
	main()


