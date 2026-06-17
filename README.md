# NOVA — Local Play‑Money Casino 🎰

A modern, single‑page casino built with **React + Vite + Tailwind CSS**. It runs
entirely in your browser — there is **no real money**, no server, and no
deposits. Accounts and balances are virtual credits saved in `localStorage`,
purely for fun and learning.

![games](https://img.shields.io/badge/games-3-2 8e98a) &nbsp; play‑money • entertainment only

## Features

- **Accounts** — register / sign in (stored locally). New players start with
  **1,000 credits**, and can claim a **+500 bonus** anytime from the navbar.
- **Persistent balance** with full bet **history** and **stats** (wagered, plays,
  biggest win), shown on the lobby.
- **Three games:**
  - **Neon Slots** — 3‑reel machine with a spinning animation and a paytable
    (up to **50×** for triple 7s).
  - **Roulette** — European single‑zero board. Bet on straight numbers, red/black,
    odd/even, low/high, dozens and columns, with a modern horizontal spin reel.
  - **Mines** — 5×5 grid; uncover gems to grow your multiplier and cash out
    before you hit a bomb. Adjustable mine count (1–24).
- **Polished UI** — dark neon theme, glassmorphism, aurora background, live‑wins
  ticker, toast notifications, and a fully **responsive** layout.

## Run it

```bash
npm install
npm run dev
```

Then open the URL Vite prints (default <http://localhost:5173>).

To build for production:

```bash
npm run build
npm run preview
```

## Tech

- React 18 + React Router 6
- Vite 5
- Tailwind CSS 3

## Project structure

```
src/
  context/      Auth (accounts + balance), Toasts, UI helpers
  components/   Navbar, Layout, AuthModal, GameHeader, BetControls, ...
  pages/        Home (lobby), Slots, Roulette, Mines
  lib/          formatting helpers + game math
```

## Note

This is a fictional, non‑commercial demo for entertainment and educational
purposes only. It does not involve real currency, prizes, or gambling, and is
not affiliated with any real operator. Passwords are stored unencrypted in the
browser — **don't reuse a real password.**
