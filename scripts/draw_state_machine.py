#!/usr/bin/env python3
"""
Filename: scripts/draw_state_machine.py
Description: CLI tool to inspect, analyze, and generate SVG state machine diagrams
    for Python StateMachine classes (e.g., hurdle.py, walking.py).

Usage examples:
    Generate an SVG diagram for the state machine
        uv run python scripts/draw_state_machine.py hurdle.py
    Specify custom output file location
        uv run python scripts/draw_state_machine.py src/hermes/aidwear/prosthesis/state_machines/walking.py -o walking.svg
    Analyze correctness and transitions in console without saving an image
        uv run python scripts/draw_state_machine.py sit_to_stand.py --analyze-only
    Save raw GraphViz `.dot` file
        uv run python scripts/draw_state_machine.py mode_selection.py --save-dot [--rankdir TB]
    Open in default viewer:
        uv run python scripts/draw_state_machine.py mode_selection.py --view
"""

import argparse
import glob
import importlib
import inspect
import os
import shutil
import sys
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Optional, Type

# Ensure project root and src/ are on sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = PROJECT_ROOT / "src"

for path_dir in [str(PROJECT_ROOT), str(SRC_DIR)]:
    if path_dir not in sys.path:
        sys.path.insert(0, path_dir)

try:
    from statemachine import StateMachine
    from statemachine.statemachine import StateChart
except ImportError:
    print(
        "Error: 'python-statemachine' is required. Install it using 'uv add python-statemachine' or 'pip install python-statemachine'.",
        file=sys.stderr,
    )
    sys.exit(1)


def find_graphviz_dot() -> Optional[str]:
    """Find the Graphviz 'dot' executable on PATH or common installation paths."""
    dot_cmd = shutil.which("dot")
    if dot_cmd:
        return dot_cmd

    # Check common Windows installation paths
    common_windows_patterns = [
        r"C:\Program Files\Graphviz*\bin\dot.exe",
        r"C:\Program Files (x86)\Graphviz*\bin\dot.exe",
        os.path.expanduser(r"~\AppData\Local\Programs\Graphviz*\bin\dot.exe"),
    ]
    for pattern in common_windows_patterns:
        matches = glob.glob(pattern)
        if matches:
            dot_path = matches[0]
            # Prepend directory to PATH so pydot finds all Graphviz plugins
            dot_dir = os.path.dirname(dot_path)
            os.environ["PATH"] = dot_dir + os.pathsep + os.environ.get("PATH", "")
            return dot_path

    # Check environment variable
    if "GRAPHVIZ_DOT" in os.environ and os.path.isfile(os.environ["GRAPHVIZ_DOT"]):
        dot_path = os.environ["GRAPHVIZ_DOT"]
        dot_dir = os.path.dirname(dot_path)
        os.environ["PATH"] = dot_dir + os.pathsep + os.environ.get("PATH", "")
        return dot_path

    return None


def resolve_file_path(target: str) -> Optional[Path]:
    """Resolve a target argument (filename, path, or partial path) to a Path."""
    p = Path(target)
    if p.is_file():
        return p.resolve()

    # If it ends without .py, check with .py
    if not target.endswith(".py"):
        p_py = Path(f"{target}.py")
        if p_py.is_file():
            return p_py.resolve()

    # Search in default state machine directory
    default_sm_dir = SRC_DIR / "hermes" / "aidwear" / "prosthesis" / "state_machines"
    candidate = default_sm_dir / (target if target.endswith(".py") else f"{target}.py")
    if candidate.is_file():
        return candidate.resolve()

    # Search in controller directory (for mode_selection.py)
    controller_dir = SRC_DIR / "hermes" / "aidwear" / "prosthesis" / "controller"
    candidate = controller_dir / (target if target.endswith(".py") else f"{target}.py")
    if candidate.is_file():
        return candidate.resolve()

    # Search recursively in src/
    filename = Path(target).name
    if not filename.endswith(".py"):
        filename = f"{filename}.py"

    matches = list(SRC_DIR.rglob(filename))
    if matches:
        return matches[0].resolve()

    return None


def import_module_from_file(file_path: Path):
    """Import a Python file as a module with parent packages correctly configured."""
    abs_file = file_path.resolve()

    # Check if the file is inside SRC_DIR or PROJECT_ROOT
    for root_candidate in [SRC_DIR, PROJECT_ROOT]:
        try:
            rel = abs_file.relative_to(root_candidate)
            # Form dotted module name
            parts = list(rel.with_suffix("").parts)
            dotted_name = ".".join(parts)
            return importlib.import_module(dotted_name)
        except (ValueError, ImportError):
            continue

    # Fallback to direct importlib loading
    module_name = abs_file.stem
    spec = importlib.util.spec_from_file_location(module_name, str(abs_file))
    if spec and spec.loader:
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
        return module

    raise ImportError(f"Could not load module from {file_path}")


def find_state_machines(
    module, class_name: Optional[str] = None
) -> list[Type[StateMachine]]:
    """Discover StateMachine subclasses defined or imported in a module."""
    machines = []
    for name, obj in inspect.getmembers(module, inspect.isclass):
        if (
            issubclass(obj, StateMachine)
            and obj is not StateMachine
            and obj is not StateChart
        ):
            # Avoid base abstract classes without states
            if hasattr(obj, "states") and len(obj.states) > 0:
                if class_name is None or name.lower() == class_name.lower():
                    machines.append(obj)

    # Prefer classes actually defined in this module
    defined_here = [m for m in machines if m.__module__ == module.__name__]
    return defined_here if defined_here else machines


def analyze_correctness(sm_cls: Type[StateMachine]) -> dict:
    """Analyze the states, transitions, and correctness checks for the state machine."""
    states = list(sm_cls.states)
    initial_states = [s for s in states if getattr(s, "initial", False)]
    final_states = [s for s in states if getattr(s, "final", False)]

    # Collect transitions
    transitions = []
    incoming_counts = {s.id: 0 for s in states}
    outgoing_counts = {s.id: 0 for s in states}
    self_loops = []

    for state in states:
        for t in state.transitions:
            if getattr(t, "internal", False):
                continue
            targets = t.targets if t.targets else [state]
            outgoing_counts[state.id] += len(targets)
            for target in targets:
                incoming_counts[target.id] += 1
                if target.id == state.id:
                    self_loops.append((state.name, t.event))

            cond_repr = [str(c) for c in getattr(t, "cond", [])]
            transitions.append(
                {
                    "source": state.name,
                    "event": t.event,
                    "targets": [target.name for target in targets],
                    "conditions": cond_repr,
                }
            )

    # Sanity checks
    issues = []
    warnings = []

    if len(initial_states) == 0:
        issues.append("No initial state defined.")
    elif len(initial_states) > 1:
        warnings.append(
            f"Multiple initial states defined: {[s.name for s in initial_states]}"
        )

    # Unreachable states (no incoming transitions and not initial)
    unreachable = [
        s.name
        for s in states
        if incoming_counts[s.id] == 0 and not getattr(s, "initial", False)
    ]
    if unreachable:
        warnings.append(
            f"Potentially unreachable states (no incoming transitions): {unreachable}"
        )

    # Trap states (non-final states with no outgoing transitions)
    trap_states = [
        s.name
        for s in states
        if outgoing_counts[s.id] == 0 and not getattr(s, "final", False)
    ]
    if trap_states:
        warnings.append(
            f"Trap states (non-final with no outgoing transitions): {trap_states}"
        )

    return {
        "class_name": sm_cls.__name__,
        "module": sm_cls.__module__,
        "states": states,
        "initial_states": initial_states,
        "final_states": final_states,
        "transitions": transitions,
        "self_loops": self_loops,
        "issues": issues,
        "warnings": warnings,
    }


def print_analysis(analysis: dict):
    """Print structured terminal analysis of the state machine."""
    print("=" * 70)
    print(f" State Machine Analysis: {analysis['class_name']}")
    print(f" Module: {analysis['module']}")
    print("=" * 70)

    print("\n[States]")
    for s in analysis["states"]:
        attrs = []
        if getattr(s, "initial", False):
            attrs.append("INITIAL")
        if getattr(s, "final", False):
            attrs.append("FINAL")
        val_str = (
            f"value={s.value}" if hasattr(s, "value") and s.value is not None else ""
        )
        attr_str = f" ({', '.join(attrs)})" if attrs else ""
        val_display = f" [{val_str}]" if val_str else ""
        print(f"  * {s.name:<18}{attr_str}{val_display}")

    print("\n[Transitions]")
    for t in analysis["transitions"]:
        targets_str = ", ".join(t["targets"])
        cond_str = f" [guards: {', '.join(t['conditions'])}]" if t["conditions"] else ""
        print(f"  * {t['source']:<16} --({t['event']})--> {targets_str:<16}{cond_str}")

    print("\n[Correctness Diagnostics]")
    if not analysis["issues"] and not analysis["warnings"]:
        print(
            "  [OK] All structural sanity checks passed (well-connected states and valid initial state)."
        )
    else:
        for issue in analysis["issues"]:
            print(f"  [ERROR] {issue}")
        for warn in analysis["warnings"]:
            print(f"  [WARN] {warn}")

    if analysis["self_loops"]:
        print(f"  [INFO] Self-loop transitions detected: {len(analysis['self_loops'])}")
    print("=" * 70)


def generate_diagram(
    sm_cls: Type[StateMachine],
    output_path: Path,
    out_format: str = "svg",
    rankdir: str = "LR",
    save_dot: bool = False,
    no_online: bool = False,
) -> Path:
    """Generate state machine diagram in SVG (or other format) using python-statemachine."""
    try:
        from statemachine.contrib.diagram import DotGraphMachine
    except ImportError as e:
        print(
            f"Error: pydot is required for diagram generation. Install it via 'uv pip install pydot'. ({e})",
            file=sys.stderr,
        )
        sys.exit(1)

    # Configure graph attributes
    DotGraphMachine.graph_rankdir = rankdir
    dot_graph = DotGraphMachine(sm_cls).get_graph()
    dot_source = dot_graph.to_string()

    # Optionally save .dot source
    if save_dot or out_format == "dot":
        dot_out = (
            output_path.with_suffix(".dot") if out_format != "dot" else output_path
        )
        dot_out.parent.mkdir(parents=True, exist_ok=True)
        dot_out.write_text(dot_source, encoding="utf-8")
        print(f"Saved Graphviz DOT source to: {dot_out}")
        if out_format == "dot":
            return dot_out

    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Check for local Graphviz 'dot'
    dot_executable = find_graphviz_dot()
    if dot_executable:
        try:
            dot_graph.write(str(output_path), format=out_format)
            print(
                f"Rendered {out_format.upper()} diagram via local Graphviz: {output_path}"
            )
            return output_path
        except Exception as e:
            print(
                f"Local Graphviz rendering failed ({e}), falling back to online rendering..."
            )

    if no_online:
        print(
            "Error: Local Graphviz 'dot' executable was not found and --no-online was specified.",
            file=sys.stderr,
        )
        dot_fallback = output_path.with_suffix(".dot")
        dot_fallback.write_text(dot_source, encoding="utf-8")
        print(f"Graphviz DOT representation saved to: {dot_fallback}", file=sys.stderr)
        sys.exit(1)

    # Fallback to QuickChart.io online rendering (statemachine's built-in fallback)
    print(
        f"Local 'dot' executable not on PATH. Rendering {out_format.upper()} diagram via QuickChart.io..."
    )
    url = f"https://quickchart.io/graphviz?format={out_format}&graph={urllib.parse.quote(dot_source)}"
    try:
        req = urllib.request.Request(
            url,
            headers={"User-Agent": "AidWear-StateMachine-Diagrammer/1.0"},
        )
        with urllib.request.urlopen(req, timeout=15) as resp:
            data = resp.read()
        output_path.write_bytes(data)
        print(f"Successfully generated diagram: {output_path}")
        return output_path
    except Exception as e:
        print(f"Online rendering request failed: {e}", file=sys.stderr)
        dot_fallback = output_path.with_suffix(".dot")
        dot_fallback.write_text(dot_source, encoding="utf-8")
        print(f"Saved raw DOT file to: {dot_fallback}", file=sys.stderr)
        print(
            "Install Graphviz locally (https://graphviz.org/download/) to render offline without external requests.",
            file=sys.stderr,
        )
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="Inspect, analyze, and generate SVG state machine diagrams for AidWear StateMachine classes.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  uv run python scripts/draw_state_machine.py hurdle.py
  uv run python scripts/draw_state_machine.py walking.py -o diagrams/walking.svg
  uv run python scripts/draw_state_machine.py sit_to_stand.py --analyze-only
  uv run python scripts/draw_state_machine.py mode_selection.py --save-dot
""",
    )
    parser.add_argument(
        "target",
        help="Path or name of the python file containing the StateMachine (e.g., hurdle.py, walking.py, or full path).",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output file path (default: <ClassName>.<format> in current working directory).",
    )
    parser.add_argument(
        "-f",
        "--format",
        default="svg",
        choices=["svg", "png", "dot", "pdf"],
        help="Output diagram format (default: svg).",
    )
    parser.add_argument(
        "-c",
        "--class-name",
        default=None,
        help="Specific StateMachine class name if multiple state machines exist in the file.",
    )
    parser.add_argument(
        "--rankdir",
        default="LR",
        choices=["LR", "TB", "RL", "BT"],
        help="Graph layout direction: LR (Left to Right, default) or TB (Top to Bottom).",
    )
    parser.add_argument(
        "--save-dot",
        action="store_true",
        help="Save the intermediate Graphviz .dot file alongside the rendered diagram.",
    )
    parser.add_argument(
        "--no-online",
        action="store_true",
        help="Disable online rendering fallback if local Graphviz 'dot' executable is missing.",
    )
    parser.add_argument(
        "--analyze-only",
        action="store_true",
        help="Analyze states, transitions, and sanity checks without generating a diagram image.",
    )
    parser.add_argument(
        "--view",
        action="store_true",
        help="Open the generated diagram in the default system viewer after generation.",
    )

    args = parser.parse_args()

    # Resolve target to file or module
    target_file = resolve_file_path(args.target)
    if target_file:
        module = import_module_from_file(target_file)
    else:
        # Try importing directly as a module name
        try:
            module = importlib.import_module(args.target)
        except ImportError as e:
            print(
                f"Error: Could not find or import target '{args.target}' (checked file paths and module names): {e}",
                file=sys.stderr,
            )
            sys.exit(1)

    # Find StateMachine classes
    machines = find_state_machines(module, class_name=args.class_name)
    if not machines:
        available_classes = [
            name
            for name, obj in inspect.getmembers(module, inspect.isclass)
            if obj.__module__ == module.__name__
        ]
        print(
            f"Error: No StateMachine subclasses with defined states found in '{args.target}'.",
            file=sys.stderr,
        )
        if available_classes:
            print(
                f"Classes found in module: {', '.join(available_classes)}",
                file=sys.stderr,
            )
        sys.exit(1)

    for sm_cls in machines:
        # Perform structural & correctness analysis
        analysis = analyze_correctness(sm_cls)
        print_analysis(analysis)

        if args.analyze_only:
            continue

        # Determine output file path
        if args.output:
            out_path = Path(args.output)
            if len(machines) > 1:
                # Disambiguate if multiple machines
                out_path = out_path.with_name(
                    f"{sm_cls.__name__.lower()}_{out_path.name}"
                )
        else:
            out_path = Path(f"{sm_cls.__name__.lower()}.{args.format}")

        result_path = generate_diagram(
            sm_cls=sm_cls,
            output_path=out_path,
            out_format=args.format,
            rankdir=args.rankdir,
            save_dot=args.save_dot,
            no_online=args.no_online,
        )

        if args.view and result_path.exists():
            import webbrowser

            webbrowser.open(result_path.as_uri())


if __name__ == "__main__":
    main()
