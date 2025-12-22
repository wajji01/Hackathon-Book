import React from 'react';
import clsx from 'clsx';
import styles from './Exercise.module.css';

interface ExerciseProps {
  children: React.ReactNode;
  title?: string;
  difficulty?: 'beginner' | 'intermediate' | 'advanced';
}

const Exercise: React.FC<ExerciseProps> = ({
  children,
  title = 'Exercise',
  difficulty = 'beginner'
}) => {
  const difficultyClass = `difficulty-${difficulty}`;

  return (
    <div className={clsx('exercise-section', styles.exercise, styles[difficultyClass])}>
      <div className={styles.exerciseHeader}>
        <h3 className={styles.exerciseTitle}>{title}</h3>
        <span className={clsx(styles.difficultyBadge, `difficulty-${difficulty}`)}>
          {difficulty.charAt(0).toUpperCase() + difficulty.slice(1)}
        </span>
      </div>
      <div className={styles.exerciseContent}>
        {children}
      </div>
    </div>
  );
};

export default Exercise;