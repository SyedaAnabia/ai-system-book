import React from "react";
import { useNavigate } from "@docusaurus/router";

export default function Protected({ children }: { children: React.ReactNode }) {
  const navigate = useNavigate();

  // Check if user is authenticated by checking for user data in localStorage
  const isAuthenticated = typeof window !== 'undefined' && localStorage.getItem("user") !== null;

  if (!isAuthenticated) {
    // Redirect to login if not authenticated
    navigate("/login");
    return null; // Return null while redirecting
  }

  return <>{children}</>;
}
