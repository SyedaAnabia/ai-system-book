import React from "react";
import { SignedIn, SignedOut, UserButton, SignInButton } from "@clerk/clerk-react";

export default function AuthButton() {
  return (
    <>
      <SignedOut>
        <SignInButton mode="modal">
          <button style={{
            padding: "6px 14px",
            borderRadius: "8px",
            background: "#2563eb",
            color: "#fff",
            border: "none",
            cursor: "pointer"
          }}>
            Login
          </button>
        </SignInButton>
      </SignedOut>

      <SignedIn>
        <UserButton />
      </SignedIn>
    </>
  );
}
