import React from 'react';

export default function ImageGallery({ items }) {
  return (
    <div
      style={{
        display: 'grid',
        gridTemplateColumns: 'repeat(auto-fit, minmax(200px, 1fr))',
        gap: '1.5rem',
        margin: '2rem 0',
      }}
    >
      {items.map((item, idx) => (
        <div
          key={idx}
          style={{
            borderRadius: '12px',
            overflow: 'hidden',
            backgroundColor: 'var(--ifm-card-background-color)',
            boxShadow: '0 4px 10px rgba(0,0,0,0.06)',
          }}
        >
          <img
            src={item.src}
            alt={item.alt}
            style={{ width: '100%', height: 'auto', display: 'block' }}
          />
          {item.description && (
            <div style={{ padding: '0.75rem', fontSize: '0.9rem' }}>
              {item.description}
            </div>
          )}
        </div>
      ))}
    </div>
  );
}

