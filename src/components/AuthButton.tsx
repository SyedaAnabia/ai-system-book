import React from "react";
import { useNavigate } from "@docusaurus/router";

export default function AuthButton() {
  const navigate = useNavigate();

  const user = typeof window !== 'undefined' ? localStorage.getItem("user") : null;
  const userData = user ? JSON.parse(user) : null;

  const handleLogout = () => {
    localStorage.removeItem("user");
    localStorage.removeItem("token");
    navigate("/"); // Redirect to home after logout
  };

  return (
    <>
      {userData ? (
        <div style={{
          display: "flex",
          alignItems: "center",
          gap: "10px"
        }}>
          <span style={{ color: "white", fontSize: "0.9rem" }}>
            Welcome, {userData.name}
          </span>
          <button
            onClick={handleLogout}
            style={{
              padding: "6px 14px",
              borderRadius: "8px",
              background: "#ef4444",
              color: "#fff",
              border: "none",
              cursor: "pointer"
            }}
          >
            Logout
          </button>
        </div>
      ) : (
        <button
          onClick={() => navigate("/login")}
          style={{
            padding: "6px 14px",
            borderRadius: "8px",
            background: "#2563eb",
            color: "#fff",
            border: "none",
            cursor: "pointer"
          }}
        >
          Login
        </button>
      )}
    </>
  );
}
