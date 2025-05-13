import { AppHeader } from '@/components/app-header';
import { SettingsForm } from '@/components/settings-form';

export default function SettingsPage() {
  return (
    <main className="flex min-h-screen flex-col items-center bg-neutral-950 text-neutral-100">
      <AppHeader title="Settings" />
      <div className="w-full p-4">
        <SettingsForm />
      </div>
    </main>
  );
}
