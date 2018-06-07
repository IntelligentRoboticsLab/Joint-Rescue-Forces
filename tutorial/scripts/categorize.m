hp_bins = [0 1 3000 7000 10000]
bin_names = {'Dead', 'Critical', 'Injured', 'Stable'}
T_train.hp_class = discretize(T_train.eHP, hp_bins, 'categorical', bin_names)