import React from 'react';

export default function ImageCard({ src, alt, description }) {
  return (
    <div style={{
      maxWidth: '500px',
      margin: '2rem auto',
      borderRadius: '12px',
      overflow: 'hidden',
      backgroundColor: 'var(--ifm-card-background-color)',
      boxShadow: '0 4px 12px rgba(0, 0, 0, 0.1)',
    }}>
      <img
        src={src}
        alt={alt}
        style={{
          width: '100%',
          height: 'auto',
          display: 'block',
        }}
      />
      <div style={{
        padding: '1rem',
        fontSize: '0.95rem',
        color: 'var(--ifm-font-color-base)',
      }}>
        {description}
      </div>
    </div>
  );
}

