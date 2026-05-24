import contextlib
import sys
import os
import fnmatch
from pathlib import Path


def read_gitignore_patterns():
    gitignore = Path(".gitignore")
    patterns = []
    if gitignore.exists():
        with open(gitignore, "r") as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#"):
                    patterns.append(line)
    return patterns


def is_ignored(path, patterns):
    try:
        relative_path = path.relative_to(Path.cwd()).as_posix()
    except ValueError:
        return False

    path_parts = relative_path.split("/")

    for pattern in patterns:
        if pattern.endswith("/"):
            dir_pattern = pattern.rstrip("/")
            if any(part == dir_pattern for part in path_parts):
                return True
        if fnmatch.fnmatch(relative_path, pattern):
            return True
        for parent in path.parents:
            try:
                parent_relative = parent.relative_to(Path.cwd()).as_posix()
                if fnmatch.fnmatch(parent_relative, pattern):
                    return True
                if pattern.endswith("/") and parent_relative == pattern.rstrip("/"):
                    return True
            except ValueError:
                continue
    return False


def collect_file_dirs():
    format_list = [".h", ".hpp", ".cpp", ".cc", ".c"]
    current_dir = Path.cwd()
    ignore_patterns = read_gitignore_patterns()
    dir_set = set()

    for file_path in current_dir.rglob("*"):
        if file_path.suffix.lower() in format_list and not is_ignored(
            file_path, ignore_patterns
        ):
            parent_dir = file_path.parent.absolute()
            dir_set.add(str(parent_dir))

    return sorted(dir_set)


def get_max_version(path):
    folders = []
    if not os.path.exists(path):
        return folders

    with contextlib.suppress(PermissionError):
        for entry in os.listdir(path):
            full_path = os.path.join(path, entry)
            if os.path.isdir(full_path):
                with contextlib.suppress(ValueError):
                    folders.append(int(entry))
    return max(folders)


def write_to_file(dirs_list, settings):
    with open(".clangd", "w") as f:
        f.write("CompileFlags:\n")
        f.write("  Add:\n")
        for i in settings[1:]:
            f.write(f'  - "{i}"\n')
        f.write(
            f'  - "-isystem/usr/include/c++/{get_max_version("/usr/include/c++")}"\n'
        )
        f.write(
            f'  - "-isystem/usr/include/x86_64-linux-gnu/c++/{get_max_version("/usr/include/x86_64-linux-gnu/c++/")}"\n'
        )
        include_dir = Path.cwd() / "include"
        third_party_dir = Path.cwd() / "third_party"
        if include_dir.is_dir():
            f.write(f'  - "-I{include_dir.absolute()}"\n')
        if third_party_dir.is_dir():
            f.write(f'  - "-I{third_party_dir.absolute()}"\n')
        for dir_path in dirs_list:
            f.write(f'  - "-I{dir_path}"\n')


if __name__ == "__main__":
    settings = sys.argv
    dir_list = collect_file_dirs()
    write_to_file(dir_list, settings)
