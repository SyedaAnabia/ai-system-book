// src/db/schema.ts
import { pgTable, text, timestamp, jsonb, serial } from 'drizzle-orm/pg-core';

export const users = pgTable('users', {
  id: serial('id').primaryKey(),
  email: text('email').notNull().unique(),
  name: text('name').notNull(),
  password: text('password').notNull(),
  
  // User Background Data (ye chatbot se collect hoga)
  userBackground: jsonb('user_background').$type<{
    softwareExperience: 'beginner' | 'intermediate' | 'advanced';
    programmingLanguages: string[];
    hasGPU: boolean;
    gpuType?: string;
    rosExperience: 'none' | 'basic' | 'intermediate' | 'advanced';
    roboticsProjects: boolean;
    roboticsDetails?: string;
    learningGoals: string;
    hardwareAccess: string[];
  }>(),
  
  createdAt: timestamp('created_at').defaultNow(),
  updatedAt: timestamp('updated_at').defaultNow(),
});

export const sessions = pgTable('sessions', {
  id: text('id').primaryKey(),
  userId: serial('user_id').references(() => users.id),
  expiresAt: timestamp('expires_at').notNull(),
  createdAt: timestamp('created_at').defaultNow(),
});