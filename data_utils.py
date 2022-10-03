class Package:

    def __init__(self, package_id) -> None:
        self.package_id = package_id
        self.package_state = 0

class Data:

    def __init__(self, data_id, data_size) -> None:
        self.data_id = data_id
        self.data_size = data_size
        
        self.packages = {}
        for i in range(data_size):
            self.packages[i] = Package(i)

    def get_not_sent_packages(self):
        not_sent_packages = []
        for package in self.packages.values():
            if package.package_state == 0:
                not_sent_packages.append(package)

        return not_sent_packages

class DataController:

    def __init__(self) -> None:
        self.datas = {}
        self.new_data_id = 0

    def add_data(self, data_size) -> Data:
        self.datas[self.new_data_id] = Data(self.new_data_id, data_size)
        self.new_data_id += 1
        return self.datas[self.new_data_id - 1]

    def get_not_sent_packages(self):
        not_sent_packages = []
        for data in self.datas.values():
            for package in data.get_not_sent_packages():
                not_sent_packages.append((data.data_id, package))

        return not_sent_packages

    def is_all_packages_delivered_or_lost(self):
        for data in self.datas.values():
            for package in data.packages.values():
                if package.package_state == 0 or package.package_state == 1:
                    return False

        return True


    