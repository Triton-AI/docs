import React from 'react';

export default function ImageGallery({ items, ratio = '1 / 1' /* square by default */ }) {
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
          {/* keeps a fixed aspect ratio for every tile */}
          <div style={{ position: 'relative', width: '100%', aspectRatio: ratio }}>
            <img
              src={item.src}
              alt={item.alt}
              style={{
                position: 'absolute',
                inset: 0,
                width: '100%',
                height: '100%',
                objectFit: 'cover',       /* crop, never stretch */
              }}
            />
          </div>

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

