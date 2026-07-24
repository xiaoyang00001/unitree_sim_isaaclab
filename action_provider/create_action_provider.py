def create_action_provider(env, args):
    """Create only the requested provider so optional backends stay optional."""
    if args.action_source == "dds":
        from action_provider.action_provider_dds import DDSActionProvider

        return DDSActionProvider(env=env, args_cli=args)
    if args.action_source == "dds_wholebody":
        from action_provider.action_provider_wh_dds import DDSRLActionProvider

        return DDSRLActionProvider(env=env, args_cli=args)
    if args.action_source == "sonic_dds":
        from action_provider.action_provider_sonic_dds import SonicDDSActionProvider

        return SonicDDSActionProvider(env=env, args_cli=args)
    if args.action_source == "replay":
        from action_provider.action_provider_replay import FileActionProviderReplay

        return FileActionProviderReplay(env=env, args_cli=args)

    print(f"unknown action source: {args.action_source}")
    return None
