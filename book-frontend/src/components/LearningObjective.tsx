import React from 'react';
import clsx from 'clsx';
import styles from './LearningObjective.module.css';

interface LearningObjectiveProps {
  children: React.ReactNode;
  title?: string;
}

const LearningObjective: React.FC<LearningObjectiveProps> = ({
  children,
  title = 'Learning Objective'
}) => {
  return (
    <div className={clsx('educational-content', 'learning-objective')}>
      <div className="learning-objective-header">
        <h3 className={styles.learningObjectiveTitle}>{title}</h3>
      </div>
      <div className={styles.learningObjectiveContent}>
        {children}
      </div>
    </div>
  );
};

export default LearningObjective;