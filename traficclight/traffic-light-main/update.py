from utils.updater import Updater
import json

if __name__ == '__main__':
    with open('config.json', 'r', encoding='utf-8') as file:
        config = json.load(file)
    updater = Updater(config['updater'])
    print('Updating...')
    try:
        updater()
        # print(updater.get_remote_hash())
        # print(updater.get_local_hash())
    except Exception as e:
        print('Update failed. Use current code.')
    else:
        print(('Update successfully.'))