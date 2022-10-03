import pandas as pd

def print_scouting_info():
    df = pd.read_csv('output/scouting_log.csv')

    print('-----SCOUTING INFO-----')
    print('Count of scounting stages:', len(df))
    print('Average count of found paths:', df['paths_count'].mean())


def print_package_info():
    df = pd.read_csv('output/data_state_log.csv')

    print('-----PACKAGE INFO-----')
    print('Count of delivered packages:', len(df[df['state'] == 2]))
    print('Count of lost packages:', len(df[df['state'] == -1]))
    print('----------')

    df_group_by_package_id = df.groupby(['package_id', 'data_id'])
    df_lifetimes = df_group_by_package_id['tick'].max() - df_group_by_package_id['tick'].min()

    print('Min lifetime of package:', df_lifetimes.min())
    print('Max lifetime of package:', df_lifetimes.max())
    print('Average lifetime of package:', df_lifetimes.mean())
    print('----------')
    

    df_group_by_data_id = df.groupby(['data_id'])
    df_lifetimes = df_group_by_data_id['tick'].max() - df_group_by_package_id['tick'].min()
    print('Min lifetime of data:', df_lifetimes.min())
    print('Max lifetime of data:', df_lifetimes.max())
    print('Average lifetime of data:', df_lifetimes.mean())
    # print('Average count of found paths:', df['paths_count'].mean())


print_scouting_info()
print()
print_package_info()