import React from 'react';
import FloatingChatbot from '../components/FloatingChatbot';

// This wraps your entire app
export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}
      <FloatingChatbot />
    </>
  );
}