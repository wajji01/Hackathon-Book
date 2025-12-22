---
id: 2
title: Update-project-constitution
stage: constitution
date: 2025-12-21
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Claude Sonnet 4.5
command: sp.constitution
labels: [constitution, update, governance]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

$ARGUMENTS

Project: AI-Spec-Driven Technical Book with Embedded RAG Chatbot

Core principles:
- Spec-first, AI-native development
- Technical accuracy (official docs only)
- Clarity for developers and CS students
- Reproducibility and deployable results

Key standards:
- Book authored with Docusaurus
- Spec-Kit Plus drives all content and structure
- Claude Code used for coding and spec alignment
- Runnable, verified code only

Technical scope:
- Deploy book to GitHub Pages
- Embedded RAG chatbot using:
  - OpenAI Agents/ChatKit SDKs
  - FastAPI backend
  - Neon Serverless Postgres
  - Qdrant Cloud (Free Tier)
- Chatbot must answer:
  - Full-book queries
  - Selected-text-only queries with source references

Constraints:
- No hallucinated APIs or features
- Stable, current SDK versions only
- Developer-level writing (intermediate+)

Success criteria:
- Successful GitHub Pages deployment
- Accurate RAG responses with citations
- Spec-driven, reproducible build from scratch

## Response snapshot

Updated the project constitution at .specify/memory/constitution.md with the following project-specific details:

1. Project name: "AI-Spec-Driven Technical Book with Embedded RAG Chatbot"
2. Core principles:
   - Spec-First, AI-Native Development
   - Technical Accuracy (Official Docs Only)
   - Clarity for Developers and CS Students
   - Reproducibility and Deployable Results
   - Spec-Driven, AI-Assisted Implementation
   - Stable, Current SDK Versions Only
3. Technical Stack and Deployment Standards
4. Development Workflow and Quality Assurance
5. Governance section with constitution authority rules

The constitution now properly reflects the project requirements specified in CLAUDE.md, including the focus on spec-first development, technical accuracy from official docs, reproducibility requirements, and the technical stack using Docusaurus, FastAPI, Neon Postgres, and Qdrant Cloud.

## Outcome

- ✅ Impact: Updated constitution to match project requirements
- 🧪 Tests: No tests required for this configuration task
- 📁 Files: Updated .specify/memory/constitution.md with project-specific principles
- 🔁 Next prompts: Project constitution is now properly configured for development
- 🧠 Reflection: Constitution now accurately represents project goals and constraints

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Not applicable