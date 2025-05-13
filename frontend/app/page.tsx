'use client';

import { CarModel } from '@/components/car-model';
import { CarInfo } from '@/components/car-info';
import { SummonButton } from '@/components/summon-button';
import { AppHeader } from '@/components/app-header';
import { OwnerInfo } from '@/components/owner-info';

export default function Home() {
  return (
    <main className="flex min-h-screen flex-col items-center bg-neutral-950 text-neutral-100">
      <AppHeader title="My Car" />

      <div className="w-full h-[40vh] md:h-[50vh] relative">
        <CarModel />
      </div>

      <div className="w-full px-4 pb-6 space-y-4">
        <OwnerInfo />
        <SummonButton />
        <CarInfo />
      </div>
    </main>
  );
}
