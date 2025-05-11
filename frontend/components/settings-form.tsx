"use client";

import { useState } from "react";
import { Switch } from "@/components/ui/switch";
import { Slider } from "@/components/ui/slider";
import { Label } from "@/components/ui/label";
import { Button } from "@/components/ui/button";
import { Bell, Wifi, Shield, Zap, ChevronRight, User } from "lucide-react";

export function SettingsForm() {
  const [notifications, setNotifications] = useState(true);
  const [wifi, setWifi] = useState(true);
  const [summonDistance, setSummonDistance] = useState(20);

  return (
    <div className="space-y-6">
      <div className="bg-neutral-900 rounded-xl p-5 shadow-lg">
        <div className="flex items-center gap-3 mb-4">
          <div className="bg-neutral-800 p-2 rounded-full">
            <User className="h-5 w-5" />
          </div>
          <div>
            <h3 className="font-medium">Illinois Student</h3>
            <p className="text-xs text-neutral-400">john.doe@illinois.edu</p>
          </div>
        </div>
        <Button variant="outline" className="w-full justify-between">
          Account Settings
          <ChevronRight className="h-4 w-4" />
        </Button>
      </div>

      <div className="bg-neutral-900 rounded-xl p-5 shadow-lg space-y-5">
        <h2 className="text-xl font-medium">Preferences</h2>

        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <Bell className="h-5 w-5 text-neutral-400" />
            <Label htmlFor="notifications">Notifications</Label>
          </div>
          <Switch
            id="notifications"
            checked={notifications}
            onCheckedChange={setNotifications}
          />
        </div>

        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <Wifi className="h-5 w-5 text-neutral-400" />
            <Label htmlFor="wifi">Wi-Fi</Label>
          </div>
          <Switch id="wifi" checked={wifi} onCheckedChange={setWifi} />
        </div>
      </div>

      <div className="bg-neutral-900 rounded-xl p-5 shadow-lg space-y-5">
        <h2 className="text-xl font-medium">Summon Settings</h2>

        <div className="space-y-3">
          <div className="flex items-center justify-between">
            <Label htmlFor="summon-distance">
              Max Distance ({summonDistance} ft)
            </Label>
          </div>
          <Slider
            id="summon-distance"
            min={5}
            max={50}
            step={1}
            value={[summonDistance]}
            onValueChange={(value) => setSummonDistance(value[0])}
          />
        </div>

        <div className="pt-2">
          <Button variant="outline" className="w-full justify-between">
            <div className="flex items-center gap-3">
              <Shield className="h-5 w-5 text-neutral-400" />
              <span>Safety & Security</span>
            </div>
            <ChevronRight className="h-4 w-4" />
          </Button>
        </div>

        <div className="pt-2">
          <Button variant="outline" className="w-full justify-between">
            <div className="flex items-center gap-3">
              <Zap className="h-5 w-5 text-neutral-400" />
              <span>Performance</span>
            </div>
            <ChevronRight className="h-4 w-4" />
          </Button>
        </div>
      </div>

      <div className="text-center text-xs text-neutral-500 pt-4">
        <p>GEMstack Summon App v1.0.0</p>
        <p className="mt-1">© 2025 All Rights Reserved</p>
      </div>
    </div>
  );
}
