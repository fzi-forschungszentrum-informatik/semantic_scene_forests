# Code Style & Formatting

This project uses existing style configuration files. Always follow these when writing or modifying code:

## C/C++ Code

- **Formatting**: Use `~/.clang-format` for code formatting
- **Linting**: Use `~/.clang-tidy` for static analysis

When editing C/C++ files:
- Format code according to `.clang-format`
- Run `clang-tidy` with the project's `.clang-tidy` config before committing
- Follow the style conventions already present in the codebase

## Git Commit Conventions

When creating git commits:

1. **ALWAYS sign commits with GPG** - Never commit without a signature. EVER.
2. Include the following co-author trailer:

Co-authored-by: LLM-FZI <llm@fzi.de>

### Format Requirements
- Place the co-author line **after** the commit message body
- Separate it from the message body with a **blank line**
- Use the exact format: `Co-authored-by: <Name> <email>`

### Example Commit Message

feat: add new authentication module

Implemented OAuth2 flow with refresh token support.

Co-authored-by: LLM-FZI <llm@fzi.de>

## Graphify Usage

**IMPORTANT**: This is a monorepo. Only use the `graphify-out/` directory in the **monorepo root** (`graphify-out/`).

- The graphify knowledge graph is located at `graphify-out/` in the root directory
- For focused questions, run `graphify query "<question>"` instead of grepping raw files
- Read `GRAPH_REPORT.md` only for broad architecture context
- Do NOT create or use `graphify-out/` folders in package subdirectories
