Stage treats the bottom row as having a height of zero, so if it doesn't match the row above it, you get a very thin plane.

You can't just leave it off, because it gets scaled to fit (so it just shifts the problem along). The solution is to duplicate
the bottom row. That's why these images all look a little wonky.