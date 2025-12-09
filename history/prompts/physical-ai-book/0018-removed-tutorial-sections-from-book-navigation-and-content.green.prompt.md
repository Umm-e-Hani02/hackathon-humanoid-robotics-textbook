---
id: 0018
title: Removed Tutorial Sections from Book Navigation and Content
stage: green
date: 2025-12-09
surface: agent
model: gemini-1.5-flash
feature: physical-ai-book
branch: main
user: admin
command: 
labels: ["docs", "cleanup", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - physical-ai-and-humanoid-robots/sidebars.js
  - physical-ai-and-humanoid-robots/docs/tutorial-basics (removed)
  - physical-ai-and-humanoid-robots/docs/tutorial-extras (removed)
tests_yaml: []
prompt_text: |
  Remove tutorial basic and tutorial extras from the book
response_text: |
  Successfully removed the "Tutorial Basics" and "Tutorial Extras" categories from the Docusaurus sidebar configuration and deleted the corresponding documentation directories.
---
