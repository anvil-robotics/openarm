"""Terminal display utilities for formatted output with cursor control."""

import sys


class TableDisplay:
    """Manage terminal table display with in-place updates using ANSI cursor control.

    This class handles rendering multi-line tables in the terminal with efficient
    in-place updates by moving the cursor and clearing lines.

    Example:
        display = TableDisplay()
        display.set_height(3)
        display.line(0, "Header")
        display.line(1, "Row 1")
        display.line(2, "Row 2")
        display.render()  # Print to terminal

        # Update in-place
        display.line(1, "Updated Row 1")
        display.render()  # Cursor moves up and updates
    """

    def __init__(self) -> None:
        """Initialize table display."""
        self._lines: list[str] = []
        self._height: int = 0
        self._first_render: bool = True

    def set_height(self, height: int) -> None:
        """Set the number of lines for the table.

        Args:
            height: Number of lines in the table.
        """
        self._height = height
        self._lines = [""] * height

    def line(self, n: int, content: str) -> None:
        """Set content for line n (0-indexed).

        Args:
            n: Line number (0-indexed).
            content: Content to display on this line.
        """
        if n < 0 or n >= self._height:
            msg = f"Line {n} out of range (0-{self._height - 1})"
            raise IndexError(msg)
        self._lines[n] = content

    def render(self) -> None:
        """Render all lines to terminal.

        On first render, prints all lines normally.
        On subsequent renders, moves cursor up and updates each line in-place.
        """
        if self._first_render:
            # First render: just print all lines
            for line in self._lines:
                sys.stdout.write(line + "\n")
            sys.stdout.flush()
            self._first_render = False
        else:
            # Subsequent renders: move cursor up and update each line
            # Move cursor up to the first line
            sys.stdout.write(f"\033[{self._height}A")

            # Print each line with carriage return and clear to end of line
            for line in self._lines:
                sys.stdout.write(f"\r{line}\033[K\n")

            sys.stdout.flush()

    def clear(self) -> None:
        """Move cursor past the table (for cleanup after final render)."""
        if not self._first_render:
            # Already rendered, cursor is below table
            pass
        sys.stdout.flush()
