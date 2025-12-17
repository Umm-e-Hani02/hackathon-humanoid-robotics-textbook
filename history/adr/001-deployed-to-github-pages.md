# ADR-001: Deployed to GitHub Pages

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-18
- **Feature:** deploy-to-github-pages
- **Context:** The project needs a free and easy way to host the Docusaurus-based book. GitHub Pages is a natural choice as the code is already hosted on GitHub.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

We will use GitHub Pages for deploying and hosting the Docusaurus book.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Free hosting.
- Tight integration with the GitHub repository.
- Simple deployment process with `npm run deploy`.
- Good performance for static sites.

### Negative

- Limited to static sites.
- Potential for vendor lock-in with GitHub ecosystem.

## Alternatives Considered

- **Vercel/Netlify:** More powerful platforms with features like serverless functions, but might be overkill for a simple static site and introduce another service to manage.
- **Self-hosting (e.g., on a VPS):** More control but also more maintenance overhead and cost.

## References

- Feature Spec: null
- Implementation Plan: null
- Related ADRs: null
- Evaluator Evidence: `history/prompts/deploy-to-github-pages/0035-deployed-docusaurus-site-to-github-pages.green.prompt.md`
