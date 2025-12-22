import React from 'react';
import clsx from 'clsx';
import styles from './ImportantNote.module.css';

interface ImportantNoteProps {
  children: React.ReactNode;
  title?: string;
}

const ImportantNote: React.FC<ImportantNoteProps> = ({
  children,
  title = 'Important Note'
}) => {
  return (
    <div className={clsx('educational-content', 'important-note')}>
      <div className={styles.importantNoteHeader}>
        <h3 className={styles.importantNoteTitle}>{title}</h3>
      </div>
      <div className={styles.importantNoteContent}>
        {children}
      </div>
    </div>
  );
};

export default ImportantNote;