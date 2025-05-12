'use client';

import Link from 'next/link';
import { usePathname } from 'next/navigation';
import { Car, Map, Settings } from 'lucide-react';
import { cn } from '@/lib/utils';
import { motion } from 'framer-motion';

export function MobileNav() {
  const pathname = usePathname();

  return (
    <motion.div
      className="fixed bottom-0 left-0 z-30 w-full h-16 bg-neutral-900 border-t border-neutral-800"
      initial={{ y: 100 }}
      animate={{ y: 0 }}
      transition={{ delay: 0.5, duration: 0.5 }}
    >
      <div className="grid h-full max-w-lg grid-cols-3 mx-auto">
        <Link
          href="/"
          className={cn(
            'inline-flex flex-col items-center justify-center px-5 hover:bg-neutral-800 transition-colors',
            pathname === '/' ? 'text-white' : 'text-neutral-400'
          )}
        >
          <motion.div whileHover={{ scale: 1.1 }} whileTap={{ scale: 0.9 }}>
            <Car className="w-6 h-6 mb-1" />
          </motion.div>
          <span className="text-xs">Car</span>
        </Link>
        <Link
          href="/map"
          className={cn(
            'inline-flex flex-col items-center justify-center px-5 hover:bg-neutral-800 transition-colors',
            pathname === '/map' ? 'text-white' : 'text-neutral-400'
          )}
        >
          <motion.div whileHover={{ scale: 1.1 }} whileTap={{ scale: 0.9 }}>
            <Map className="w-6 h-6 mb-1" />
          </motion.div>
          <span className="text-xs">Map</span>
        </Link>
        <Link
          href="/settings"
          className={cn(
            'inline-flex flex-col items-center justify-center px-5 hover:bg-neutral-800 transition-colors',
            pathname === '/settings' ? 'text-white' : 'text-neutral-400'
          )}
        >
          <motion.div whileHover={{ scale: 1.1 }} whileTap={{ scale: 0.9 }}>
            <Settings className="w-6 h-6 mb-1" />
          </motion.div>
          <span className="text-xs">Settings</span>
        </Link>
      </div>
    </motion.div>
  );
}
