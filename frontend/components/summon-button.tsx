'use client';

import { useRouter } from 'next/navigation';
import { Button } from '@/components/ui/button';
import { ArrowUpCircle } from 'lucide-react';
import { motion } from 'framer-motion';
import { toast } from 'sonner';

export function SummonButton() {
  const router = useRouter();

  const handleSummon = () => {
    toast('Redirecting to map', {
      description: 'Opening map to summon your vehicle',
    });

    // Redirect to map page
    router.push('/map');
  };

  return (
    <Button
      className="w-full h-14 rounded-xl text-lg font-medium"
      onClick={handleSummon}
    >
      <motion.div className="flex items-center gap-2">
        <ArrowUpCircle className="h-6 w-6" />
        <span>Summon Car</span>
      </motion.div>
    </Button>
  );
}
