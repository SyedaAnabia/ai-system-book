import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import FloatingChatbot from '../components/HomepageFeatures/FloatingChatbot';
import Translator from '../components/Translator';

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <BrowserOnly>
      {() => {
        const { ClerkProvider } = require('@clerk/clerk-react');

        return (
          <ClerkProvider publishableKey="pk_test_Z3JhdGVmdWwtbmFyd2hhbC02My5jbGVyay5hY2NvdW50cy5kZXYk">
            {children}

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

            <FloatingChatbot />
          </ClerkProvider>
        );
      }}
    </BrowserOnly>
  );
}
