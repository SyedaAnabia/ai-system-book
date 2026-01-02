import React, { useState } from "react";
import Link from '@docusaurus/Link';

export default function Login() {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError("");

    try {
      const response = await fetch("/api/auth/login", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (response.ok) {
        // Store user data in localStorage or session storage
        localStorage.setItem("user", JSON.stringify(data.user));
        localStorage.setItem("token", "dummy-token"); // In a real app, you'd use the actual token from the backend

        // Redirect to home page or dashboard using window.location
        window.location.href = "/";
      } else {
        setError(data.detail || "Login failed. Please check your credentials.");
      }
    } catch (err) {
      console.error("Login error:", err);
      setError("An error occurred during login. Please try again.");
    } finally {
      setLoading(false);
    }
  };

  return (
    <div style={{
      display: "flex",
      justifyContent: "center",
      alignItems: "center",
      minHeight: "80vh",
      marginTop: "40px",
      padding: "20px"
    }}>
      <form
        onSubmit={handleSubmit}
        style={{
          width: "100%",
          maxWidth: "400px",
          padding: "30px",
          background: "white",
          borderRadius: "10px",
          boxShadow: "0 10px 25px rgba(0,0,0,0.1)",
        }}
      >
        <h2 style={{
          textAlign: "center",
          marginBottom: "25px",
          color: "#1e293b",
          fontSize: "1.8rem",
          fontWeight: "700"
        }}>
          Login to Your Account
        </h2>

        {error && (
          <div style={{
            backgroundColor: "#fee2e2",
            color: "#dc2626",
            padding: "10px",
            borderRadius: "5px",
            marginBottom: "15px",
            fontSize: "0.9rem"
          }}>
            {error}
          </div>
        )}

        <div style={{ marginBottom: "20px" }}>
          <label htmlFor="email" style={{
            display: "block",
            marginBottom: "8px",
            fontWeight: "600",
            color: "#334155"
          }}>
            Email Address
          </label>
          <input
            id="email"
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            style={{
              width: "100%",
              padding: "12px",
              border: "1px solid #cbd5e1",
              borderRadius: "6px",
              fontSize: "1rem",
            }}
          />
        </div>

        <div style={{ marginBottom: "20px" }}>
          <label htmlFor="password" style={{
            display: "block",
            marginBottom: "8px",
            fontWeight: "600",
            color: "#334155"
          }}>
            Password
          </label>
          <input
            id="password"
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            style={{
              width: "100%",
              padding: "12px",
              border: "1px solid #cbd5e1",
              borderRadius: "6px",
              fontSize: "1rem",
            }}
          />
        </div>

        <button
          type="submit"
          disabled={loading}
          style={{
            width: "100%",
            padding: "14px",
            backgroundColor: loading ? "#94a3b8" : "#2563eb",
            color: "white",
            border: "none",
            borderRadius: "6px",
            fontSize: "1.1rem",
            fontWeight: "600",
            cursor: loading ? "not-allowed" : "pointer",
            transition: "background-color 0.3s",
          }}
          onMouseEnter={(e) => !loading && (e.currentTarget.style.backgroundColor = "#1d4ed8")}
          onMouseLeave={(e) => !loading && (e.currentTarget.style.backgroundColor = "#2563eb")}
        >
          {loading ? "Logging in..." : "Login"}
        </button>

        <div style={{
          textAlign: "center",
          marginTop: "20px",
          fontSize: "0.95rem"
        }}>
          Don't have an account?{" "}
          <a
            href="/register"
            style={{
              color: "#2563eb",
              textDecoration: "none",
              fontWeight: "600"
            }}
          >
            Register here
          </a>
        </div>
      </form>
    </div>
  );
}
