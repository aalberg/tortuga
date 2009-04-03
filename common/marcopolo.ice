module ram
{
	module marcopolo
	{
		sequence<int> IntList;
		
		struct Snapshot
		{
			IntList samples;
		};
		
		class Polo
		{
			void reportData(Snapshot snap);
		};
		
		class Marco
		{
			idempotent void setChannelEnabled(int channel, bool enabled);
			idempotent bool getChannelEnabled(int channel);
			idempotent void setTrigger(int channel, int level);
			void registerPolo(Polo* p);
		};
	};
};