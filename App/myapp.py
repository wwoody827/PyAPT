import argparse
parser = argparse.ArgumentParser(prog='PROG', description='description')
parser.add_argument('cmd', choices=['load','test','config','quit'])


while True:
    astr = raw_input('$: ')
    # print astr
    try:
        args = parser.parse_args(astr.split())
    except SystemExit:
        # trap argparse error message
        print 'error'
        continue

    if args.cmd in ['read', 'delete']:
        print 'doing', args.cmd
    elif args.cmd == 'help':
        parser.print_help()
    else:
        print 'done'
        break