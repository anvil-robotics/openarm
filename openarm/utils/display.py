"""Terminal display utilities for formatted output with cursor control."""

import sys


class Display:
    """Manage multi-line terminal display with in-place updates using ANSI cursor control.

    This class handles rendering multiple lines in the terminal with efficient
    in-place updates by moving the cursor and clearing lines.

    Example:
        display = Display()
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
        """Initialize display."""
        self._lines: list[str] = []
        self._height: int = 0
        self._first_render: bool = True

    def set_height(self, height: int) -> None:
        """Set the number of lines for the display.

        Args:
            height: Number of lines in the display.
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
        """Move cursor past the display (for cleanup after final render)."""
        if not self._first_render:
            # Already rendered, cursor is below display
            pass
        sys.stdout.flush()


class TableDisplay:
    """Table display with automatic column formatting and truncation.

    This class wraps a Display instance and provides table-like formatting with
    defined column widths. Cells are automatically truncated if they exceed the
    specified column width.

    Example:
        display = Display()
        display.set_height(3)
        table = TableDisplay(display, columns_length=[10, 20, 15])
        table.row(0, ["Name", "Description", "Value"])
        table.row(1, ["Item1", "A very long description that will be cut", "123"])
        table.row(2, ["Item2", "Short", "456"])
        display.render()
    """

    def __init__(self, display: Display, columns_length: list[int]) -> None:
        """Initialize table display.

        Args:
            display: Display instance to render to.
            columns_length: List of column widths (in characters).
        """
        self._display = display
        self._columns_length = columns_length

    def row(self, n: int, cells: list[str]) -> None:
        """Set content for row n with automatic column formatting.

        Args:
            n: Row number (0-indexed).
            cells: List of cell contents (one per column).
        """
        # Format each cell according to column width
        formatted_cells = []
        for i, cell in enumerate(cells):
            if i < len(self._columns_length):
                col_width = self._columns_length[i]
                # Truncate if cell exceeds column width
                if len(cell) > col_width:
                    formatted_cell = cell[:col_width]
                else:
                    formatted_cell = cell
                formatted_cells.append(formatted_cell)
            else:
                # No width defined for this column, use as-is
                formatted_cells.append(cell)

        # Join cells into a single line
        line = "".join(formatted_cells)
        self._display.line(n, line)
