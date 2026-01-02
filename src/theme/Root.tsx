import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import FloatingChatbot from '../components/HomepageFeatures/FloatingChatbot';
import Translator from '../components/Translator';

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}

      <BrowserOnly>
        {() => (
          <div
            style={{
              position: "fixed",
              bottom: "20px",
              left: "20px",
              zIndex: 9999,
              background: "#020617",
              padding: "6px 10px",
              borderRadius: "10px",
              boxShadow: "0 0 10px rgba(0,0,0,0.4)",
            }}
          >
            <Translator />
          </div>
        )}
      </BrowserOnly>

      <BrowserOnly>
        {() => <FloatingChatbot />}
      </BrowserOnly>
    </>
  );
}
