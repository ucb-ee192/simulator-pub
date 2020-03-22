# simulator-pub
V-REP simulator

## Developer Resources

### Static checking
The majority of this code has mypy static type annotations.
If you have mypy installed (can be done via `pip`) you can typecheck the code using:   
```
dmypy run -- --follow-imports=error --disallow-untyped-defs --disallow-incomplete-defs --check-untyped-defs -p carTest-slew20-line
```
