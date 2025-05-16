import React from 'react';

export default function ImageCard({ src, alt, description }) {
  return (
    <div style={{
      width: '100%', // Full width
      maxWidth: '100%', // Ensures the card doesn't go beyond its container
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
          height: 'auto', // Maintains aspect ratio
          display: 'block',
          objectFit: 'contain', // Ensures the whole image is visible without clipping
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

