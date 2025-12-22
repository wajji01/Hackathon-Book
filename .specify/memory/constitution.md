<!--
Sync Impact Report:
Version change: N/A → 1.0.0
Modified principles: None (new constitution)
Added sections: All principles and sections (new constitution)
Removed sections: None
Templates requiring updates: ✅ plan-template.md (already references constitution checks), ✅ spec-template.md (aligns with requirements), ✅ tasks-template.md (aligns with task organization)
Follow-up TODOs: None
-->
# AI-Spec-Driven Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-First, AI-Native Development
All development starts with clear specifications; AI tools (Claude Code, Spec-Kit Plus) drive content and structure; Requirements are precise and testable before implementation

### Technical Accuracy (Official Docs Only)
All technical content must be based on official documentation only; No hallucinated APIs or features; All code examples must be verified and runnable

### Clarity for Developers and CS Students
Content must be clear and accessible to both practicing developers and computer science students; Explanations should be intermediate+ level with sufficient detail for understanding

### Reproducibility and Deployable Results
All code examples and setups must be reproducible from scratch; Deployments must be successful and verifiable; Full-book queries and selected-text-only queries with source references must work

### Spec-Driven, AI-Assisted Implementation
Use Spec-Kit Plus to drive all content and structure; Claude Code for coding and spec alignment; Maintain tight coupling between specs and implementation

### Stable, Current SDK Versions Only
Use only stable, current SDK versions; Avoid experimental or beta features that may break reproducibility; Verify compatibility across all components

## Technical Stack and Deployment Standards
Book authored with Docusaurus; Deploy to GitHub Pages; Embedded RAG chatbot using OpenAI Agents/ChatKit SDKs, FastAPI backend, Neon Serverless Postgres, Qdrant Cloud (Free Tier)

## Development Workflow and Quality Assurance
All code must be runnable and verified; No hallucinated APIs or features; Developer-level writing (intermediate+); Strict adherence to official documentation

## Governance
Constitution supersedes all other practices; Amendments require documentation and approval; All implementations must comply with core principles; Use official documentation as the authoritative source

**Version**: 1.0.0 | **Ratified**: 2025-12-21 | **Last Amended**: 2025-12-21
