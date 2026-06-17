/** @type {import('tailwindcss').Config} */
export default {
  content: ['./index.html', './src/**/*.{js,jsx}'],
  theme: {
    extend: {
      colors: {
        ink: '#06080f',
        night: '#0a0d17',
        surface: '#111626',
        panel: '#161c2e',
        panel2: '#1e2640',
        line: 'rgba(255,255,255,0.08)',
        neon: { DEFAULT: '#28e98a', soft: '#67ffba', deep: '#0f9d63' },
        gold: { DEFAULT: '#ffce5c', deep: '#e0a93a' },
        violet: { DEFAULT: '#8a6cff', soft: '#b6a4ff' },
        rose: { DEFAULT: '#ff4f6d', soft: '#ff8ea1' },
        sky: { DEFAULT: '#41d4ff' },
        felt: { red: '#dc3a52', black: '#1b2236', green: '#19a96c' },
      },
      fontFamily: {
        display: ['"Space Grotesk"', 'ui-sans-serif', 'system-ui', 'sans-serif'],
        sans: ['Inter', 'ui-sans-serif', 'system-ui', 'sans-serif'],
      },
      keyframes: {
        float: {
          '0%,100%': { transform: 'translate(0,0) scale(1)' },
          '50%': { transform: 'translate(30px,-40px) scale(1.08)' },
        },
        floatB: {
          '0%,100%': { transform: 'translate(0,0) scale(1)' },
          '50%': { transform: 'translate(-40px,30px) scale(1.12)' },
        },
        pop: {
          '0%': { transform: 'scale(0.7)', opacity: '0' },
          '60%': { transform: 'scale(1.06)', opacity: '1' },
          '100%': { transform: 'scale(1)' },
        },
        rise: {
          '0%': { transform: 'translateY(12px)', opacity: '0' },
          '100%': { transform: 'translateY(0)', opacity: '1' },
        },
        shimmer: {
          '0%': { backgroundPosition: '-200% 0' },
          '100%': { backgroundPosition: '200% 0' },
        },
        glowpulse: {
          '0%,100%': { boxShadow: '0 0 22px rgba(40,233,138,0.35)' },
          '50%': { boxShadow: '0 0 40px rgba(40,233,138,0.7)' },
        },
        shake: {
          '0%,100%': { transform: 'translateX(0)' },
          '20%,60%': { transform: 'translateX(-6px)' },
          '40%,80%': { transform: 'translateX(6px)' },
        },
        ticker: {
          '0%': { transform: 'translateX(0)' },
          '100%': { transform: 'translateX(-50%)' },
        },
      },
      animation: {
        float: 'float 14s ease-in-out infinite',
        floatB: 'floatB 18s ease-in-out infinite',
        pop: 'pop 0.35s ease-out',
        rise: 'rise 0.5s ease-out both',
        shimmer: 'shimmer 2.5s linear infinite',
        glowpulse: 'glowpulse 2s ease-in-out infinite',
        shake: 'shake 0.4s ease-in-out',
        ticker: 'ticker 40s linear infinite',
      },
    },
  },
  plugins: [],
}
