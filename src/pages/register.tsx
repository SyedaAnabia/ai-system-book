import React, { useState } from "react";
import Link from '@docusaurus/Link';

export default function Register() {
  const [formData, setFormData] = useState({
    name: "",
    email: "",
    password: "",
    softwareExperience: "",
    programmingLanguages: [] as string[],
    hasGPU: false,
    gpuType: "",
    rosExperience: "",
    roboticsProjects: false,
    roboticsDetails: "",
    learningGoals: "",
    hardwareAccess: [] as string[],
  });
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement | HTMLSelectElement>) => {
    const { name, value, type } = e.target;
    const checked = (e.target as HTMLInputElement).checked;

    if (type === "checkbox") {
      setFormData({
        ...formData,
        [name]: name === "programmingLanguages" || name === "hardwareAccess"
          ? value
          : checked
      });
    } else {
      setFormData({
        ...formData,
        [name]: value
      });
    }
  };

  const handleArrayChange = (e: React.ChangeEvent<HTMLInputElement>, field: string) => {
    const value = e.target.value;
    const checked = e.target.checked;

    setFormData(prev => ({
      ...prev,
      [field]: checked
        ? [...prev[field as keyof typeof formData] as string[], value]
        : (prev[field as keyof typeof formData] as string[]).filter(item => item !== value)
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError("");

    try {
      const response = await fetch("/api/auth/register", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(formData),
      });

      const data = await response.json();

      if (response.ok) {
        // Store user data in localStorage or session storage
        localStorage.setItem("user", JSON.stringify(data.user));
        localStorage.setItem("token", "dummy-token");

        // Redirect to home page or dashboard using window.location
        window.location.href = "/";
      } else {
        setError(data.detail || "Registration failed. Please try again.");
      }
    } catch (err) {
      console.error("Registration error:", err);
      setError("An error occurred during registration. Please try again.");
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
          maxWidth: "600px",
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
          Create Your Account
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
          <label htmlFor="name" style={{
            display: "block",
            marginBottom: "8px",
            fontWeight: "600",
            color: "#334155"
          }}>
            Full Name
          </label>
          <input
            id="name"
            name="name"
            type="text"
            value={formData.name}
            onChange={handleChange}
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
            name="email"
            type="email"
            value={formData.email}
            onChange={handleChange}
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
            name="password"
            type="password"
            value={formData.password}
            onChange={handleChange}
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
          <label htmlFor="softwareExperience" style={{
            display: "block",
            marginBottom: "8px",
            fontWeight: "600",
            color: "#334155"
          }}>
            Software Experience
          </label>
          <select
            id="softwareExperience"
            name="softwareExperience"
            value={formData.softwareExperience}
            onChange={handleChange}
            required
            style={{
              width: "100%",
              padding: "12px",
              border: "1px solid #cbd5e1",
              borderRadius: "6px",
              fontSize: "1rem",
            }}
          >
            <option value="">Select your experience level</option>
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>

        <div style={{ marginBottom: "20px" }}>
          <label style={{
            display: "block",
            marginBottom: "8px",
            fontWeight: "600",
            color: "#334155"
          }}>
            Programming Languages
          </label>
          {["Python", "C++", "Java", "JavaScript", "ROS", "Other"].map(lang => (
            <label key={lang} style={{ display: "block", marginBottom: "8px" }}>
              <input
                type="checkbox"
                name="programmingLanguages"
                value={lang}
                checked={formData.programmingLanguages.includes(lang)}
                onChange={(e) => handleArrayChange(e, "programmingLanguages")}
                style={{ marginRight: "8px" }}
              />
              {lang}
            </label>
          ))}
        </div>

        <div style={{ marginBottom: "20px" }}>
          <label style={{
            display: "flex",
            alignItems: "center",
            marginBottom: "8px",
            fontWeight: "600",
            color: "#334155"
          }}>
            <input
              type="checkbox"
              name="hasGPU"
              checked={formData.hasGPU}
              onChange={handleChange}
              style={{ marginRight: "8px" }}
            />
            Do you have access to a GPU?
          </label>
        </div>

        {formData.hasGPU && (
          <div style={{ marginBottom: "20px" }}>
            <label htmlFor="gpuType" style={{
              display: "block",
              marginBottom: "8px",
              fontWeight: "600",
              color: "#334155"
            }}>
              GPU Type
            </label>
            <input
              id="gpuType"
              name="gpuType"
              type="text"
              value={formData.gpuType}
              onChange={handleChange}
              style={{
                width: "100%",
                padding: "12px",
                border: "1px solid #cbd5e1",
                borderRadius: "6px",
                fontSize: "1rem",
              }}
            />
          </div>
        )}

        <div style={{ marginBottom: "20px" }}>
          <label htmlFor="rosExperience" style={{
            display: "block",
            marginBottom: "8px",
            fontWeight: "600",
            color: "#334155"
          }}>
            ROS Experience
          </label>
          <select
            id="rosExperience"
            name="rosExperience"
            value={formData.rosExperience}
            onChange={handleChange}
            required
            style={{
              width: "100%",
              padding: "12px",
              border: "1px solid #cbd5e1",
              borderRadius: "6px",
              fontSize: "1rem",
            }}
          >
            <option value="">Select your ROS experience</option>
            <option value="none">No Experience</option>
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>

        <div style={{ marginBottom: "20px" }}>
          <label style={{
            display: "flex",
            alignItems: "center",
            marginBottom: "8px",
            fontWeight: "600",
            color: "#334155"
          }}>
            <input
              type="checkbox"
              name="roboticsProjects"
              checked={formData.roboticsProjects}
              onChange={handleChange}
              style={{ marginRight: "8px" }}
            />
            Have you worked on robotics projects?
          </label>
        </div>

        {formData.roboticsProjects && (
          <div style={{ marginBottom: "20px" }}>
            <label htmlFor="roboticsDetails" style={{
              display: "block",
              marginBottom: "8px",
              fontWeight: "600",
              color: "#334155"
            }}>
              Details about your robotics projects
            </label>
            <textarea
              id="roboticsDetails"
              name="roboticsDetails"
              value={formData.roboticsDetails}
              onChange={handleChange}
              rows={3}
              style={{
                width: "100%",
                padding: "12px",
                border: "1px solid #cbd5e1",
                borderRadius: "6px",
                fontSize: "1rem",
              }}
            />
          </div>
        )}

        <div style={{ marginBottom: "20px" }}>
          <label htmlFor="learningGoals" style={{
            display: "block",
            marginBottom: "8px",
            fontWeight: "600",
            color: "#334155"
          }}>
            Learning Goals
          </label>
          <textarea
            id="learningGoals"
            name="learningGoals"
            value={formData.learningGoals}
            onChange={handleChange}
            required
            rows={3}
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
          <label style={{
            display: "block",
            marginBottom: "8px",
            fontWeight: "600",
            color: "#334155"
          }}>
            Hardware Access
          </label>
          {["Arduino", "Raspberry Pi", "Robot Arm", "Mobile Robot", "Sensors", "Other"].map(hw => (
            <label key={hw} style={{ display: "block", marginBottom: "8px" }}>
              <input
                type="checkbox"
                name="hardwareAccess"
                value={hw}
                checked={formData.hardwareAccess.includes(hw)}
                onChange={(e) => handleArrayChange(e, "hardwareAccess")}
                style={{ marginRight: "8px" }}
              />
              {hw}
            </label>
          ))}
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
          {loading ? "Registering..." : "Register"}
        </button>

        <div style={{
          textAlign: "center",
          marginTop: "20px",
          fontSize: "0.95rem"
        }}>
          Already have an account?{" "}
          <a
            href="/login"
            style={{
              color: "#2563eb",
              textDecoration: "none",
              fontWeight: "600"
            }}
          >
            Login here
          </a>
        </div>
      </form>
    </div>
  );
}