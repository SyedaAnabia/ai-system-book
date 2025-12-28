import React from 'react';
import { ClerkProvider } from "@clerk/clerk-react";
import FloatingChatbot from '../components/HomepageFeatures/FloatingChatbot';
import Translator from '../components/Translator';

// Browser mein window se load hoga, SSG ke time undefined hoga
const getClerkKey = () => {
  if (typeof window !== 'undefined') {
    return (window as any).CLERK_PUBLISHABLE_KEY;
  }
  return undefined;
};

export default function Root({ children }: { children: React.ReactNode }) {
  const clerkKey = getClerkKey();

  // SSG time (server-side) - bina Clerk ke render karo
  if (typeof window === 'undefined') {
    return <>{children}</>;
  }

  // Browser mein - agar key nahi hai toh warning do aur bina auth ke chalao
  if (!clerkKey) {
    console.warn('⚠️ Clerk key not found. Add it to static/env.js');
    return (
      <>
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
      </>
    );
  }

  // Browser mein with Clerk
  return (
    <ClerkProvider publishableKey={clerkKey}>
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
}
