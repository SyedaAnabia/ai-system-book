import type { ReactNode } from 'react';
import { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import FloatingChatbot from "../components/FloatingChatbot";


import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  const [scrolled, setScrolled] = useState(false);
  const [activeFeature, setActiveFeature] = useState(0);

  useEffect(() => {
    const handleScroll = () => setScrolled(window.scrollY > 50);
    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  useEffect(() => {
    const interval = setInterval(() => {
      setActiveFeature((prev) => (prev + 1) % 3);
    }, 3000);
    return () => clearInterval(interval);
  }, []);

  const features = [
    { emoji: "📘", title: "Comprehensive", color: "from-blue-500 to-cyan-500" },
    { emoji: "🎯", title: "Practical", color: "from-purple-500 to-pink-500" },
    { emoji: "🚀", title: "Progressive", color: "from-orange-500 to-red-500" }
  ];

  return (
    <header className={clsx('hero', styles.heroBanner)} style={{
      position: 'relative',
      background: 'linear-gradient(135deg, #0f172a 0%, #1e293b 50%, #334155 100%)',
      minHeight: '100vh',
      display: 'flex',
      alignItems: 'center',
      overflow: 'hidden',
      padding: '2rem 0'
    }}>
      {/* Animated Background Blobs */}
      <div style={{ position: 'absolute', inset: 0, opacity: 0.3 }}>
        <div style={{
          position: 'absolute',
          top: '20%',
          left: '10%',
          width: '300px',
          height: '300px',
          background: '#3b82f6',
          borderRadius: '50%',
          filter: 'blur(80px)',
          animation: 'pulse 4s ease-in-out infinite'
        }}></div>
        <div style={{
          position: 'absolute',
          bottom: '20%',
          right: '10%',
          width: '400px',
          height: '400px',
          background: '#8b5cf6',
          borderRadius: '50%',
          filter: 'blur(80px)',
          animation: 'pulse 4s ease-in-out infinite 1s'
        }}></div>
        <div style={{
          position: 'absolute',
          top: '50%',
          left: '50%',
          transform: 'translate(-50%, -50%)',
          width: '250px',
          height: '250px',
          background: '#06b6d4',
          borderRadius: '50%',
          filter: 'blur(80px)',
          animation: 'pulse 4s ease-in-out infinite 2s'
        }}></div>
      </div>

      <div className="container" style={{ position: 'relative', zIndex: 10 }}>
        <div style={{
          display: 'grid',
          gridTemplateColumns: '1fr',
          gap: '4rem',
          alignItems: 'center',
          maxWidth: '1400px',
          margin: '0 auto'
        }}>
          {/* Left Content */}
          <div style={{ textAlign: 'center' }}>
            <div style={{
              display: 'inline-block',
              padding: '0.5rem 1.5rem',
              background: 'rgba(6, 182, 212, 0.1)',
              border: '1px solid rgba(6, 182, 212, 0.3)',
              borderRadius: '50px',
              fontSize: '0.9rem',
              fontWeight: '600',
              color: '#06b6d4',
              marginBottom: '2rem'
            }}>
              ✨ Interactive Learning Experience
            </div>

            <Heading as="h1" style={{
              fontSize: 'clamp(2.5rem, 8vw, 5rem)',
              fontWeight: '900',
              marginBottom: '1.5rem',
              color: 'white',
              lineHeight: '1.2',
              textShadow: '2px 2px 8px rgba(0,0,0,0.3)'
            }}>
              {siteConfig.title}
            </Heading>

            <div style={{
              fontSize: 'clamp(1.8rem, 5vw, 3rem)',
              fontWeight: '800',
              background: 'linear-gradient(135deg, #06b6d4 0%, #3b82f6 50%, #8b5cf6 100%)',
              backgroundClip: 'text',
              WebkitBackgroundClip: 'text',
              WebkitTextFillColor: 'transparent',
              backgroundSize: '200% 200%',
              animation: 'gradient 3s ease infinite',
              marginBottom: '2rem'
            }}>
              Master Your Knowledge
            </div>

            <p style={{
              fontSize: 'clamp(1.1rem, 2vw, 1.4rem)',
              color: 'rgba(255,255,255,0.85)',
              maxWidth: '800px',
              margin: '0 auto 3rem',
              lineHeight: '1.8',
              textShadow: '1px 1px 2px rgba(0,0,0,0.2)'
            }}>
              {siteConfig.tagline}
            </p>

            {/* Buttons */}
            <div style={{
              display: 'flex',
              gap: '1.5rem',
              justifyContent: 'center',
              flexWrap: 'wrap',
              marginBottom: '3rem'
            }}>
              <Link
                to="/docs/intro"  // Start Reading
                style={{
                  display: 'inline-flex',
                  alignItems: 'center',
                  gap: '0.5rem',
                  padding: '1rem 2.5rem',
                  background: 'linear-gradient(135deg, #06b6d4 0%, #3b82f6 100%)',
                  color: 'white',
                  fontWeight: '700',
                  fontSize: '1.1rem',
                  borderRadius: '50px',
                  border: 'none',
                  boxShadow: '0 10px 30px rgba(6, 182, 212, 0.4)',
                  transition: 'all 0.3s ease',
                  textDecoration: 'none'
                }}
                onMouseEnter={(e) => {
                  e.currentTarget.style.transform = 'scale(1.05)';
                  e.currentTarget.style.boxShadow = '0 15px 40px rgba(6, 182, 212, 0.6)';
                }}
                onMouseLeave={(e) => {
                  e.currentTarget.style.transform = 'scale(1)';
                  e.currentTarget.style.boxShadow = '0 10px 30px rgba(6, 182, 212, 0.4)';
                }}>
                📖 Start Reading
                <span style={{ fontSize: '1.2rem' }}>→</span>
              </Link>

              <Link
                to="/docs/intro" // Browse Modules also points to intro
                style={{
                  display: 'inline-flex',
                  alignItems: 'center',
                  gap: '0.5rem',
                  padding: '1rem 2.5rem',
                  background: 'rgba(255,255,255,0.1)',
                  color: 'white',
                  fontWeight: '700',
                  fontSize: '1.1rem',
                  borderRadius: '50px',
                  border: '2px solid rgba(255,255,255,0.3)',
                  backdropFilter: 'blur(10px)',
                  boxShadow: '0 8px 20px rgba(0,0,0,0.2)',
                  transition: 'all 0.3s ease',
                  textDecoration: 'none'
                }}
                onMouseEnter={(e) => {
                  e.currentTarget.style.background = 'rgba(255,255,255,0.2)';
                  e.currentTarget.style.transform = 'scale(1.05)';
                }}
                onMouseLeave={(e) => {
                  e.currentTarget.style.background = 'rgba(255,255,255,0.1)';
                  e.currentTarget.style.transform = 'scale(1)';
                }}>
                📚 Browse Modules
              </Link>
            </div>
          </div>
        </div>
      </div>

      <style>{`
        @keyframes pulse {
          0%, 100% { opacity: 0.3; transform: scale(1); }
          50% { opacity: 0.5; transform: scale(1.1); }
        }

        @keyframes gradient {
          0% { background-position: 0% 50%; }
          50% { background-position: 100% 50%; }
          100% { background-position: 0% 50%; }
        }
      `}</style>
    </header>
  );
}

function FeaturesSection() {
  const modules = [
    { number: "01", title: "Module 1", desc: "Introduction to fundamental concepts and core principles", gradient: "from-blue-500 to-cyan-500" },
    { number: "02", title: "Module 2", desc: "Building on basics with advanced techniques and methods", gradient: "from-purple-500 to-pink-500" },
    { number: "03", title: "Module 3", desc: "Applying knowledge through practical implementations", gradient: "from-orange-500 to-red-500" },
    { number: "04", title: "Module 4", desc: "Mastering advanced topics and real-world applications", gradient: "from-green-500 to-emerald-500" }
  ];

  return (
    <section style={{ padding: '6rem 2rem', background: 'linear-gradient(180deg, #1e293b 0%, #0f172a 100%)', position: 'relative' }}>
      <div className="container" style={{ maxWidth: '1400px', margin: '0 auto' }}>
        <div style={{ textAlign: 'center', marginBottom: '4rem' }}>
          <h2 style={{ fontSize: 'clamp(2rem, 5vw, 3.5rem)', fontWeight: '800', marginBottom: '1rem', color: 'white' }}>
            Course{' '}
            <span style={{ background: 'linear-gradient(135deg, #06b6d4 0%, #3b82f6 100%)', backgroundClip: 'text', WebkitBackgroundClip: 'text', WebkitTextFillColor: 'transparent' }}>
              Modules
            </span>
          </h2>
          <p style={{ fontSize: '1.3rem', color: 'rgba(255,255,255,0.7)', maxWidth: '600px', margin: '0 auto' }}>
            Comprehensive learning path structured for success
          </p>
        </div>

        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(280px, 1fr))', gap: '2.5rem', maxWidth: '1200px', margin: '0 auto' }}>
          {modules.map((module, index) => (
            <Link
              key={index}
              to="/docs/intro" // Updated to intro
              style={{
                padding: '3rem 2rem',
                background: 'rgba(255,255,255,0.05)',
                borderRadius: '20px',
                border: '1px solid rgba(255,255,255,0.1)',
                backdropFilter: 'blur(10px)',
                transition: 'all 0.4s ease',
                position: 'relative',
                overflow: 'hidden',
                textDecoration: 'none',
                display: 'block',
                cursor: 'pointer'
              }}
            >
              {/* Module Badge */}
              <div style={{
                position: 'absolute',
                top: '1.5rem',
                right: '1.5rem',
                width: '50px',
                height: '50px',
                background: `linear-gradient(135deg, ${module.gradient.split(' ')[1]} 0%, ${module.gradient.split(' ')[3]} 100%)`,
                borderRadius: '50%',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                fontSize: '1.5rem',
                fontWeight: '900',
                color: 'white',
                boxShadow: '0 8px 20px rgba(0,0,0,0.3)',
                border: '3px solid rgba(255,255,255,0.2)'
              }}>{index + 1}</div>

              {/* Module Icon */}
              <div style={{
                width: '90px',
                height: '90px',
                background: `linear-gradient(135deg, ${module.gradient.split(' ')[1]} 0%, ${module.gradient.split(' ')[3]} 100%)`,
                borderRadius: '20px',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                marginBottom: '1.5rem',
                boxShadow: '0 15px 40px rgba(0,0,0,0.4)',
                fontSize: '3rem',
                position: 'relative',
                overflow: 'hidden'
              }}>
                <div style={{ position: 'absolute', inset: 0, background: 'rgba(255,255,255,0.1)', borderRadius: '20px' }}></div>
                📚
              </div>

              <h3 style={{ fontSize: '2rem', fontWeight: '800', marginBottom: '0.5rem', color: 'white', letterSpacing: '-0.5px' }}>{module.title}</h3>
              <div style={{ width: '50px', height: '4px', background: `linear-gradient(90deg, ${module.gradient.split(' ')[1]} 0%, ${module.gradient.split(' ')[3]} 100%)`, borderRadius: '2px', marginBottom: '1rem' }}></div>
              <p style={{ color: 'rgba(255,255,255,0.75)', lineHeight: '1.8', fontSize: '1.05rem', marginBottom: '2rem' }}>{module.desc}</p>

              <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: '1rem' }}>
                <div style={{
                  display: 'inline-flex',
                  alignItems: 'center',
                  gap: '0.5rem',
                  padding: '0.85rem 1.75rem',
                  background: `linear-gradient(135deg, ${module.gradient.split(' ')[1]} 0%, ${module.gradient.split(' ')[3]} 100%)`,
                  borderRadius: '50px',
                  color: 'white',
                  fontWeight: '700',
                  fontSize: '1rem',
                  boxShadow: '0 8px 20px rgba(0,0,0,0.3)',
                  transition: 'all 0.3s ease'
                }}>
                  Start Learning
                  <span style={{ fontSize: '1.3rem' }}>→</span>
                </div>

                <div style={{ fontSize: '0.9rem', color: 'rgba(255,255,255,0.5)', fontWeight: '600' }}>{3 + index} Weeks</div>
              </div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description={`${siteConfig.tagline} - Learn through comprehensive modules and guided content`}>
      <HomepageHeader />
      <main>
        <FeaturesSection />
      </main>
      <FloatingChatbot />

    </Layout>
  );
}
